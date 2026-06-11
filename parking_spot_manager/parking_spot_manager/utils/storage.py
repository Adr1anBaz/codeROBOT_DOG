import os
import yaml
import logging
from typing import List, Optional

from .parking_spot import ParkingSpot

logger = logging.getLogger(__name__)


class ParkingSpotStorage:

    def __init__(self, base_directory: str = ''):
        self.base_directory = base_directory

    def get_spots_file_path(self, map_name: str) -> str:
        clean_name = os.path.splitext(os.path.basename(map_name))[0]
        filename = f"{clean_name}_parking_spots.yaml"
        if self.base_directory:
            return os.path.join(self.base_directory, filename)
        return filename

    def load_spots(self, map_name: str) -> List[ParkingSpot]:
        file_path = self.get_spots_file_path(map_name)
        if not os.path.exists(file_path):
            logger.warning(f"Parking spots file not found: {file_path}")
            return []

        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)

            if data is None or 'parking_spots' not in data:
                return []

            spots = []
            for spot_data in data['parking_spots']:
                spots.append(ParkingSpot.from_dict(spot_data))
            return spots

        except Exception as e:
            logger.error(f"Error loading parking spots from {file_path}: {e}")
            return []

    def save_spots(self, map_name: str, spots: List[ParkingSpot]) -> bool:
        file_path = self.get_spots_file_path(map_name)
        try:
            directory = os.path.dirname(file_path)
            if directory:
                os.makedirs(directory, exist_ok=True)

            data = {
                'map_name': map_name,
                'spot_count': len(spots),
                'parking_spots': [spot.to_dict() for spot in spots],
            }

            with open(file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)

            logger.info(f"Saved {len(spots)} parking spots to {file_path}")
            return True

        except Exception as e:
            logger.error(f"Error saving parking spots to {file_path}: {e}")
            return False

    def append_spot(self, map_name: str, spot: ParkingSpot) -> bool:
        spots = self.load_spots(map_name)
        spots.append(spot)
        return self.save_spots(map_name, spots)

    def spot_exists(self, map_name: str, spot_name: str) -> bool:
        spots = self.load_spots(map_name)
        return any(s.name == spot_name for s in spots)

    def get_spot_by_id(self, map_name: str, spot_id: int) -> Optional[ParkingSpot]:
        spots = self.load_spots(map_name)
        if 0 <= spot_id < len(spots):
            return spots[spot_id]
        return None

    def get_spot_by_name(self, map_name: str, name: str) -> Optional[ParkingSpot]:
        spots = self.load_spots(map_name)
        for spot in spots:
            if spot.name == name:
                return spot
        return None
