import { MapContainer, TileLayer, Polyline, CircleMarker, Popup, useMap } from "react-leaflet";
import { useEffect, useMemo } from "react";

const CHENNAI_CENTER = [13.0827, 80.2707];
const CHENNAI_BOUNDS = [
  [12.75, 79.95],
  [13.35, 80.45]
];

function MapAutoView({ liveFrame, trail, sourceHospital, destinationHospital }) {
  const map = useMap();

  useEffect(() => {
    const points = [];
    if (sourceHospital?.latitude != null && sourceHospital?.longitude != null) {
      points.push([sourceHospital.latitude, sourceHospital.longitude]);
    }
    if (destinationHospital?.latitude != null && destinationHospital?.longitude != null) {
      points.push([destinationHospital.latitude, destinationHospital.longitude]);
    }
    if (Array.isArray(trail) && trail.length > 0) {
      points.push(...trail);
    }
    if (liveFrame?.lat != null && liveFrame?.lon != null) {
      points.push([Number(liveFrame.lat), Number(liveFrame.lon)]);
    }

    if (points.length >= 2) {
      map.fitBounds(points, { padding: [30, 30], maxZoom: 13 });
    } else if (points.length === 1) {
      map.setView(points[0], 12);
    } else {
      map.fitBounds(CHENNAI_BOUNDS, { padding: [24, 24] });
    }
  }, [map, liveFrame, trail, sourceHospital, destinationHospital]);

  return null;
}

export default function LiveMissionMap({ liveFrame, trail, sourceHospital, destinationHospital }) {
  const uavPosition = useMemo(() => {
    if (liveFrame?.lat == null || liveFrame?.lon == null) {
      return null;
    }
    return [Number(liveFrame.lat), Number(liveFrame.lon)];
  }, [liveFrame]);

  const defaultCenter = useMemo(() => {
    if (uavPosition) {
      return uavPosition;
    }
    if (sourceHospital?.latitude != null && sourceHospital?.longitude != null) {
      return [sourceHospital.latitude, sourceHospital.longitude];
    }
    if (destinationHospital?.latitude != null && destinationHospital?.longitude != null) {
      return [destinationHospital.latitude, destinationHospital.longitude];
    }
    return CHENNAI_CENTER;
  }, [uavPosition, sourceHospital, destinationHospital]);

  return (
    <div className="live-map-root">
      <MapContainer center={defaultCenter} zoom={11} className="live-map">
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />

        <MapAutoView
          liveFrame={liveFrame}
          trail={trail}
          sourceHospital={sourceHospital}
          destinationHospital={destinationHospital}
        />

        {trail?.length > 1 && <Polyline positions={trail} pathOptions={{ color: "#4ec2ff", weight: 4 }} />}

        {sourceHospital?.latitude != null && sourceHospital?.longitude != null && (
          <CircleMarker
            center={[sourceHospital.latitude, sourceHospital.longitude]}
            radius={8}
            pathOptions={{ color: "#3dfc8f", fillColor: "#3dfc8f", fillOpacity: 0.8 }}
          >
            <Popup>Source Hospital</Popup>
          </CircleMarker>
        )}

        {destinationHospital?.latitude != null && destinationHospital?.longitude != null && (
          <CircleMarker
            center={[destinationHospital.latitude, destinationHospital.longitude]}
            radius={8}
            pathOptions={{ color: "#ff6ec7", fillColor: "#ff6ec7", fillOpacity: 0.8 }}
          >
            <Popup>Destination Hospital</Popup>
          </CircleMarker>
        )}

        {uavPosition && (
          <CircleMarker
            center={uavPosition}
            radius={7}
            pathOptions={{ color: "#00d4ff", fillColor: "#00d4ff", fillOpacity: 1 }}
          >
            <Popup>UAV Position</Popup>
          </CircleMarker>
        )}
      </MapContainer>
    </div>
  );
}
