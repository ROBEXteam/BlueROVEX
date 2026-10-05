import folium

# Chemin vers ton fichier
file_path = "/home/tbelier/Documents/GIT/BlueROVEX/ros2_quick_start_bluerov_ws/rosbag2_2026_04_16-16_01_23/ekf_latlonelev_only.csv"  # adapte si besoin

points = []

# Lecture du fichier
with open(file_path, "r") as f:
    lines = f.readlines()

# Ignorer l'en-tête
for line in lines[1:]:
    parts = line.strip().split(",")
    
    lat = float(parts[0])
    lon = float(parts[1])
    
    points.append((lat, lon))

# Centre de la carte
map_center = points[0]

# Carte satellite
m = folium.Map(
    location=map_center,
    zoom_start=20,      # zoom initial plus proche
    max_zoom=25,        # 👈 autorise zoom très fort
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="Esri"
)

m = folium.Map(location=map_center, zoom_start=18)

# Trajectoire (ligne)
folium.PolyLine(
    points,
    color="red",
    weight=3,
    opacity=0.8
).add_to(m)

# Sauvegarde
m.save("map_trajectory.html")

print("Carte avec trajectoire générée : map_trajectory.html")