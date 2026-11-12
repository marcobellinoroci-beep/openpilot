#!/usr/bin/env python3
import os

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.api import API_HOST, Api
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.route import along_path, local_meters

REROUTE_DISTANCE = 25
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
ROUTE_RESEND_PERIOD = 10  # updates (seconds); a restarted mapsd gets the route within this
RECOMPUTE_HOLD = 10  # updates (seconds) between route requests
MAPBOX_HOST = "https://api.mapbox.com"
COMMA_NAV_HOST = API_HOST.rstrip('/') + '/v1/nav'
GEOCODE_RETRY = 30  # updates (seconds) before retrying a geocode after a network error or missing token
GEOCODE_LANGUAGE = 'it'
GEOCODE_MAX_QUERY = 256  # Mapbox limit: 256 characters, no ';'


class GeocodeNoResult(Exception):
  pass


def geocode(query: str, token: str, proximity: tuple[float, float] | None = None, language: str = GEOCODE_LANGUAGE):
  """Forward-geocode a typed address/place with Mapbox Geocoding v6. Returns (latitude, longitude, name).

  Raises GeocodeNoResult when nothing matches, requests exceptions on network/HTTP errors.
  """
  q = query.replace(';', ' ').strip()[:GEOCODE_MAX_QUERY]
  params = {'q': q, 'access_token': token, 'limit': 1, 'autocomplete': 'false', 'language': language}
  if proximity is not None:
    params['proximity'] = f"{proximity[0]:.5f},{proximity[1]:.5f}"
  resp = requests.get(MAPBOX_HOST + '/search/geocode/v6/forward', params=params, timeout=10)
  resp.raise_for_status()
  features = resp.json().get('features') or []
  if not features:
    raise GeocodeNoResult(q)
  feature = features[0]
  props = feature.get('properties') or {}
  coords = props.get('coordinates') or {}
  # Prefer the routable (vehicle stop) point when Mapbox gives one.
  routable = next((p for p in coords.get('routable_points') or [] if p.get('name') == 'default'), None)
  if routable is not None:
    lon, lat = routable['longitude'], routable['latitude']
  elif 'longitude' in coords and 'latitude' in coords:
    lon, lat = coords['longitude'], coords['latitude']
  else:
    lon, lat = feature['geometry']['coordinates'][:2]
  name = props.get('full_address') or props.get('name') or q
  return float(lat), float(lon), str(name)


def set_banner(instruction, banners, distance_to_maneuver):
  """Fill the banner fields from the banner to show now: the last one still ahead of the ego, else the first."""
  if not banners:
    return
  banner = next((b for b in reversed(banners) if distance_to_maneuver < b['distanceAlongGeometry']), banners[0])
  instruction.showFull = distance_to_maneuver < banner['distanceAlongGeometry']
  primary = banner['primary']
  instruction.maneuverPrimaryText = primary.get('text') or ''
  instruction.maneuverType = primary.get('type') or ''
  instruction.maneuverModifier = primary.get('modifier') or ''
  if banner.get('secondary') is not None:
    instruction.maneuverSecondaryText = banner['secondary']['text']


class RouteEngine:
  def __init__(self, sm, pm, params=None):
    self.sm = sm
    self.pm = pm
    self.params = params if params is not None else Params()

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step

    self.recompute_hold = 0
    self.reroute_counter = 0

    self.mapbox_token = self.get_mapbox_token()
    self.api = Api(self.params.get("DongleId")) if self.mapbox_token is None else None
    self.mapbox_host = MAPBOX_HOST if self.mapbox_token is not None else COMMA_NAV_HOST

    # Destination typed on the device (NavDestinationPending), see update_pending_destination
    self.geocode_hold = 0
    self.applied_pending_key = None

  def get_mapbox_token(self) -> str | None:
    """Own Mapbox token: MAPBOX_TOKEN env var first, then the MapboxToken param (set from the device UI)."""
    token = os.environ.get("MAPBOX_TOKEN")
    if not token:
      try:
        token = self.params.get("MapboxToken")
      except Exception:
        token = None
    token = token.strip() if isinstance(token, str) else None
    return token or None

  def update(self):
    self.sm.update(0)

    source = latest_gps_source(self.sm)
    if source is not None:
      gps = self.sm[source]
      self.last_position, self.last_bearing = (gps.longitude, gps.latitude), gps.bearingDeg
    route = self.route
    try:
      self.update_pending_destination()
    except Exception:
      cloudlog.exception("navd.failed_pending_destination")
    try:
      self.recompute_route(gps_ok=source is not None)
      self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")
    # mapsd dedupes repeats, so a periodic resend is how a restarted mapsd gets the route
    if self.route is not route or self.sm.frame % ROUTE_RESEND_PERIOD == 0:
      self.send_route()

  def _write_pending(self, query: str, **fields):
    """Write the geocode result/status back, unless the user typed a new destination meanwhile."""
    current = self.params.get("NavDestinationPending")
    if not isinstance(current, dict) or current.get('query') != query:
      return
    new = {'query': query, **fields}
    if new != current:
      self.params.put("NavDestinationPending", new)

  def update_pending_destination(self):
    """Geocode the destination typed on the device and copy it into NavDestination.

    NavDestinationPending survives manager restarts and offroad transitions (NavDestination does not):
    {'query', 'status': 'pending'|'ok'|'error', 'message', and when ok 'name', 'latitude', 'longitude'}.
    """
    pending = self.params.get("NavDestinationPending")
    if not isinstance(pending, dict) or not str(pending.get('query') or '').strip():
      self.applied_pending_key = None
      return
    query = str(pending['query'])
    status = pending.get('status')

    if status == 'pending':
      if self.geocode_hold > 0:
        self.geocode_hold -= 1
        return
      token = self.get_mapbox_token()
      if token is None:
        self.geocode_hold = GEOCODE_RETRY
        self._write_pending(query, status='pending', message='Token Mapbox mancante: impostalo in Sviluppatore > Mapbox Token')
        return
      try:
        lat, lon, name = geocode(query, token, proximity=self.last_position)
      except GeocodeNoResult:
        self._write_pending(query, status='error', message='Nessun risultato trovato')
        return
      except requests.exceptions.HTTPError as e:
        code = e.response.status_code if e.response is not None else 0
        self.geocode_hold = GEOCODE_RETRY
        msg = 'Token Mapbox non valido' if code in (401, 403) else f'Errore Mapbox (HTTP {code})'
        self._write_pending(query, status='pending', message=msg)
        return
      except (requests.exceptions.RequestException, ValueError, KeyError, TypeError, IndexError):
        cloudlog.exception("navd.geocode_failed")
        self.geocode_hold = GEOCODE_RETRY
        self._write_pending(query, status='pending', message='Errore di rete, nuovo tentativo tra 30 s')
        return
      cloudlog.warning(f"Geocoded destination: {name}")
      self._write_pending(query, status='ok', message='', name=name, latitude=lat, longitude=lon)
      # Apply right away (same update) if the result was stored.
      pending = self.params.get("NavDestinationPending")
      if not isinstance(pending, dict) or pending.get('query') != query:
        return
      status = pending.get('status')

    if status == 'ok':
      try:
        dest = {'latitude': float(pending['latitude']), 'longitude': float(pending['longitude'])}
      except (KeyError, TypeError, ValueError):
        return
      key = (query, dest['latitude'], dest['longitude'])
      current = self.params.get("NavDestination")
      # Apply when nothing is set (e.g. after offroad/manager start) or when a new destination was typed.
      if current is None or key != self.applied_pending_key:
        if current is None or (current.get('latitude'), current.get('longitude')) != (dest['latitude'], dest['longitude']):
          self.params.put("NavDestination", dest)
        self.applied_pending_key = key

  def recompute_route(self, gps_ok: bool):
    if self.last_position is None:
      return

    destination = self.params.get("NavDestination")
    if destination is None:
      self.clear_route()
      self.recompute_hold = 0
      return
    new_destination = (destination['longitude'], destination['latitude'])

    # Don't recompute when GPS drifts in tunnels
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    if (self.should_recompute() or new_destination != self.nav_destination) and self.recompute_hold <= 0:
      self.recompute_hold = RECOMPUTE_HOLD
      self.calculate_route(new_destination)
      self.reroute_counter = 0

  def calculate_route(self, destination):
    assert self.last_position is not None
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.nav_destination = destination

    lang = self.params.get('LanguageSetting')
    if lang is not None:
      lang = lang.replace('main_', '')

    # Re-read so a token entered after navd started is used on the next route request.
    token = self.get_mapbox_token()
    self.mapbox_token = token
    if token is None:
      if self.api is None:
        self.api = Api(self.params.get("DongleId"))
      token = self.api.get_token()
    self.mapbox_host = MAPBOX_HOST if self.mapbox_token is not None else COMMA_NAV_HOST

    params = {
      'access_token': token,
      'geometries': 'geojson',
      'overview': 'full',
      'steps': 'true',
      'banner_instructions': 'true',
      'alternatives': 'false',
      'language': lang,
    }
    if self.last_bearing is not None:
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90;"

    coords_str = ';'.join(f'{lon},{lat}' for lon, lat in (self.last_position, destination))
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      if len(r['routes']):
        self.route = r['routes'][0]['legs'][0]['steps']
        self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
        self.step_idx = 0
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")
      self.clear_route()

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)

    if self.step_idx is None:
      msg.valid = False
      self.pm.send('navInstruction', msg)
      return

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    step = self.route[self.step_idx]
    along_geometry, _ = along_path(self.route_geometry[self.step_idx], self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    set_banner(msg.navInstruction, banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)

    # Total remaining distance and time: the rest of this step, then every step after it
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    future = self.route[self.step_idx + 1:]
    msg.navInstruction.distanceRemaining = step['distance'] * remaining + sum(s['distance'] for s in future)
    msg.navInstruction.timeRemaining = step['duration'] * remaining + sum(s['duration'] for s in future)

    self.pm.send('navInstruction', msg)

    # Transition to next route segment
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.recompute_hold = 0
      else:
        cloudlog.warning("Destination reached")

        # Clear route if driving away from destination
        if np.linalg.norm(local_meters(self.last_position, self.nav_destination)) > REROUTE_DISTANCE:
          self.params.remove("NavDestination")
          self.clear_pending_destination(self.nav_destination)
          self.clear_route()

  def send_route(self):
    coords = np.concatenate(self.route_geometry) if self.route_geometry is not None else np.empty((0, 2))
    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = [{'longitude': lon, 'latitude': lat} for lon, lat in coords.tolist()]
    self.pm.send('navRoute', msg)

  def clear_pending_destination(self, destination):
    """On arrival, drop the device-typed destination so it is not applied again."""
    pending = self.params.get("NavDestinationPending")
    if not isinstance(pending, dict) or pending.get('status') != 'ok' or destination is None:
      return
    try:
      if (float(pending['longitude']), float(pending['latitude'])) == (float(destination[0]), float(destination[1])):
        self.params.remove("NavDestinationPending")
        self.applied_pending_key = None
    except (KeyError, TypeError, ValueError, IndexError):
      pass

  def clear_route(self):
    self.route = None
    self.route_geometry = None
    self.step_idx = None
    self.nav_destination = None

  def should_recompute(self):
    if self.step_idx is None:
      return True

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False

    # TODO: Check for going wrong way in segment
    if along_path(self.route_geometry[self.step_idx], self.last_position)[1] > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute'])
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
