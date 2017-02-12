import numpy as np;
import utm
import xml.etree.ElementTree
import os


fname='best_particle_fast.xml'
fname_base = os.path.splitext(fname)[0]
root = xml.etree.ElementTree.parse(fname).getroot()

def generateKML(kfile, points, raw_gps=[]):
    kfile.seek(0)

    kfile.write(
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">\n'
    '<Document>\n'
    '	<name>')
    kfile.write(fname)
    kfile.write(
    '</name>\n'
    '	<Placemark>\n'
    '		<name>'+fname+'_matricies</name>\n'
    '		<styleUrl>#failed0</styleUrl>\n'
    '		<LineString>\n'
    '			<tessellate>1</tessellate>\n'
    '			<coordinates>\n')
    kfile.flush()
    for point in points:
         s = "%s,%s,0 "% (point[1],point[0],)
         kfile.write(s)
    kfile.write(
    '			</coordinates>\n'
    '           <Linestyle><color>ffff00ff</color></Linestyle>\n'
    '		</LineString>\n'
    '	</Placemark>\n'
    '	<Placemark>\n'
    '		<name>'+fname+'_raw_gps</name>\n'
    '		<styleUrl>#m_ylw-pushpin</styleUrl>\n'
    '		<LineString>\n'
    '			<tessellate>1</tessellate>\n'
    '			<coordinates>\n')
    kfile.flush()
    for point in raw_gps:
         s = "%s,%s,0 "% (point[1],point[0],)
         kfile.write(s)
    kfile.write(
    '			</coordinates>\n'
    '		</LineString>\n'
    '	</Placemark>\n'
    '</Document>\n'
    '</kml>'
    )
    kfile.flush()


def stringToNp(st):
    splitted = st.split(' ')

    return np.array([
        [float(splitted[0]), float(splitted[1]),  float(splitted[2]),  float(splitted[3])],
        [float(splitted[4]), float(splitted[5]),  float(splitted[6]),  float(splitted[7])],
        [float(splitted[8]), float(splitted[9]),  float(splitted[10]), float(splitted[11])],
        [float(splitted[12]),float(splitted[13]), float(splitted[14]), float(splitted[15])]])

# a = np.array([[-0.555648,   -0.831417,         -0, 5.78264e+06],
#   [-0.831417,    0.555648,           0,      494569],
#   [     -0,           0,          -1,           0],
#   [      0,           0,           0,           1]])

georeferenceMatrix = stringToNp(root.find("Georeference").find('Affine').find('Data').text);
georeferenceMatrix = georeferenceMatrix.T
zoneNumber = int(root.find("Georeference").find("zoneNumber").text)
zoneLetter = root.find("Georeference").find("zoneLetter").text
print ("georeferenceMatrix", georeferenceMatrix)
print ("zoneNumber", zoneNumber)
print ("zoneLetter", zoneLetter)

coordinates_map=[]
coordinates_gps=[]
for t in root.find('Transformations'):
    gps_str = t.find("gps").text
    if gps_str is not None:
        #print (gps_str)
        d = gps_str.split()
        ll1 = d[0]
        ll2 = d[1]
        ll3 = d[2]

        coordinates_gps.append([ll1,ll2,ll3])
    m = t.find('Affine').find('Data').text
    mm = stringToNp(m)
    v = np.array([mm[3,0],mm[3,1],mm[3,2],1])
    b = georeferenceMatrix.dot(v)
    #print (b)
    (lat,lon) = utm.to_latlon(b[1], b[0], zone_number=zoneNumber, zone_letter=zoneLetter);
    coordinates_map.append([lat,lon])



kfile = open (fname_base+"_map.kml", 'w')
generateKML(kfile, coordinates_map, coordinates_gps)
