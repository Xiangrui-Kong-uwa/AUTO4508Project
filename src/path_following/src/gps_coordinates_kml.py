import xml.etree.ElementTree as ET

def extract_coordinates_from_kml(kml_file):
    tree = ET.parse(kml_file)
    root = tree.getroot()

    coordinates = []

    # Find all <coordinates> elements
    for element in root.iter('{http://www.opengis.net/kml/2.2}coordinates'):
        # Extract and split the coordinate values
        coords = element.text.strip().split()
        
        # Process each coordinate value
        for coord in coords:
            # Split the coordinate string into latitude, longitude, and optional altitude
            parts = coord.split(',')
            
            # Append the latitude and longitude to the coordinates array
            latitude = float(parts[1])
            longitude = float(parts[0])
            coordinates.append((latitude, longitude))

    return coordinates

# Example usage
kml_file_path = 'Pioneer-coordinates.kml'
coordinates = extract_coordinates_from_kml(kml_file_path)