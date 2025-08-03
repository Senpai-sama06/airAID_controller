import geopandas as gpd
from shapely.geometry import Polygon, Point, box, LineString
from shapely.affinity import rotate
import matplotlib.pyplot as plt

# --------------------------------------------
# 1. Load and prepare polygon from KML
# --------------------------------------------
def load_polygon_from_kml(kml_path, target_epsg=32643):
    gdf = gpd.read_file(kml_path, driver="KML")
    geom = gdf.geometry.iloc[0]

    if geom.geom_type == 'MultiPoint':
        points = list(geom.geoms)
    elif geom.geom_type == 'Point':
        points = [geom]
    elif geom.geom_type == 'Polygon':
        points = list(geom.exterior.coords)
    else:
        raise ValueError(f"Unsupported geometry type: {geom.geom_type}")

    if len(points) < 3:
        raise ValueError("At least 3 points are needed to form a polygon")

    poly = Polygon(points).convex_hull

    # Set and convert CRS
    gdf = gdf.set_crs(epsg=4326)
    gdf = gdf.to_crs(epsg=target_epsg)
    poly = gpd.GeoSeries([poly], crs=4326).to_crs(epsg=target_epsg).iloc[0]

    return poly, gdf.crs

# --------------------------------------------
# 2. Divide polygon into horizontal strips
# --------------------------------------------
def divide_polygon_into_strips(poly, strip_width, angle_deg=0):
    minx, miny, maxx, maxy = poly.bounds
    diag_len = ((maxx - minx)*2 + (maxy - miny)*2) * 0.5
    num_strips = int(diag_len / strip_width) + 2
    strips = []

    for i in range(-num_strips, num_strips):
        y_offset = miny + i * strip_width
        rect = box(minx - diag_len, y_offset, maxx + diag_len, y_offset + strip_width)
        rotated = rotate(rect, angle_deg, origin='center', use_radians=False)
        inter = poly.intersection(rotated)
        if not inter.is_empty:
            if inter.geom_type == 'Polygon':
                strips.append(inter)
            elif inter.geom_type == 'MultiPolygon':
                strips.extend(inter.geoms)
    return strips

# --------------------------------------------
# 3. Assign priority based on water intersection
# --------------------------------------------
def assign_priority(strips, water_shp_path, crs):
    gdf_strips = gpd.GeoDataFrame(geometry=strips, crs=crs)
    water_layer = gpd.read_file(water_shp_path).to_crs(crs)

    gdf_strips["has_water"] = gdf_strips.geometry.apply(
        lambda geom: not water_layer.empty and geom.intersects(water_layer.unary_union)
    )
    gdf_strips["priority_group"] = gdf_strips["has_water"].apply(lambda x: 1 if x else 2)
    gdf_strips["centroid_y"] = gdf_strips.geometry.centroid.y

    # Sort by priority and top-down order
    gdf_strips = gdf_strips.sort_values(by=["priority_group", "centroid_y"], ascending=[True, False]).reset_index(drop=True)
    gdf_strips["strip_order"] = range(1, len(gdf_strips) + 1)

    return gdf_strips

# --------------------------------------------
# 4. Get midpoints of strip edges
# --------------------------------------------
def get_strip_edge_midpoints(geom):
    minx, miny, maxx, maxy = geom.bounds
    return Point(minx, (miny + maxy) / 2), Point(maxx, (miny + maxy) / 2)

# --------------------------------------------
# 5. Generate zig-zag path through strips
# --------------------------------------------
def generate_path(gdf_strips):
    start_pts, end_pts = [], []
    for geom in gdf_strips.geometry:
        left, right = get_strip_edge_midpoints(geom)
        start_pts.append(left)
        end_pts.append(right)

    path_coords = []
    current_point = None

    for i in range(len(gdf_strips)):
        left = start_pts[i]
        right = end_pts[i]

        if current_point is None:
            path_coords.extend([(left.x, left.y), (right.x, right.y)])
            current_point = right
        else:
            if current_point.distance(left) < current_point.distance(right):
                path_coords.extend([(left.x, left.y), (right.x, right.y)])
                current_point = right
            else:
                path_coords.extend([(right.x, right.y), (left.x, left.y)])
                current_point = left

    return path_coords

# --------------------------------------------
# 6. Plot result
# --------------------------------------------
def plot_strips_and_path(gdf_strips, path_coords):
    fig, ax = plt.subplots(figsize=(10, 10))
    gdf_strips.plot(ax=ax, facecolor='none', edgecolor='black', linewidth=1.5)

    for _, row in gdf_strips.iterrows():
        centroid = row.geometry.centroid
        ax.text(centroid.x, centroid.y, f"{row['strip_order']}", fontsize=8, ha='center', va='center', color='blue')

    line = LineString([Point(xy) for xy in path_coords])
    ax.plot(*line.xy, color='red', linewidth=2, marker='o', markersize=5, label='Path')

    plt.title("Strips and prioritized path")
    plt.legend()
    plt.axis('equal')
    plt.show()

# --------------------------------------------
# 7. Main pipeline function
# --------------------------------------------
def get_path(kml_path, water_shp_path, strip_width=50, angle_deg=0, target_epsg=32643):
    poly, crs = load_polygon_from_kml(kml_path, target_epsg)
    strips = divide_polygon_into_strips(poly, strip_width, angle_deg)
    gdf_strips = assign_priority(strips, water_shp_path, crs)
    path_coords = generate_path(gdf_strips)

    # Convert path to GeoDataFrame and WGS84
    gdf_path = gpd.GeoDataFrame(geometry=[Point(xy) for xy in path_coords], crs=crs)
    gdf_path_wgs84 = gdf_path.to_crs(epsg=4326)
    coords_wgs84 = [(pt.x, pt.y) for pt in gdf_path_wgs84.geometry]
    
    print("\nPath coordinates (lon, lat) in order:")
    for lon, lat in coords_wgs84:
        print(f"{lon}, {lat}")

    # Plot
    plot_strips_and_path(gdf_strips, path_coords)

    return coords_wgs84
coords = get_path(
    kml_path="new_area.kml",
    water_shp_path="Water_Polygons.shp",
    strip_width=50,
    angle_deg=0,
    target_epsg=32643  # UTM zone, adjust based on location

    )

print(coords)