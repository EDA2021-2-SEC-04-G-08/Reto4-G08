"""
 * Copyright 2020, Departamento de sistemas y Computación,
 * Universidad de Los Andes
 *
 *
 * Desarrolado para el curso ISIS1225 - Estructuras de Datos y Algoritmos
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along withthis program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contribuciones:
 *
 * Dario Correal - Version inicial
 """



import config
from DISClib.ADT.graph import gr
from DISClib.ADT import stack as stk 
from DISClib.ADT import list as lt
from DISClib.ADT import map as mp
from DISClib.ADT import orderedmap as om
from DISClib.DataStructures import mapentry as me
from DISClib.Algorithms.Graphs import scc
from DISClib.Algorithms.Sorting import mergesort as ms
from DISClib.Algorithms.Graphs import dfs
from DISClib.Algorithms.Graphs import dijsktra as djk
from DISClib.Algorithms.Graphs import prim 
assert config
from math import dist, radians, cos, sin, asin, sqrt

"""
En este archivo definimos los TADs que vamos a usar y las operaciones
de creacion y consulta sobre las estructuras de datos.
"""

#=======================================================================
# Construccion de modelos
#=======================================================================

def newAnalyzer():
    """ Inicializa el analizador
    airports: tabla de hash para guardar los vertices del grafo
    red: grafo para representar los vuelos entre aeropuertos
    """
    
    analyzer = {
                'red': None,
                'aeropuertos': None,
                "components": None
                }

    analyzer['red'] = gr.newGraph(datastructure='ADJ_LIST',
                                              directed=True,
                                              size=14000,
                                              comparefunction=compara_mapa)
    analyzer['blue'] = gr.newGraph(datastructure='ADJ_LIST',
                                              directed=False,
                                              size=14000,
                                              comparefunction=compara_mapa)                                       
    analyzer['aeropuertos'] = mp.newMap(comparefunction=compara_mapa)
    analyzer['ciudades'] = mp.newMap(comparefunction=compara_mapa)
    # mapa homonimos req3
    analyzer['ciudades_id'] = mp.newMap(comparefunction=compara_mapa)
    # mapa ordenado req3
    analyzer['latlng'] = om.newMap() 

    return analyzer

#======================================================================
def contar_ciudades(map):
    total=0
    for i in lt.iterator(mp.valueSet(map)):
        total+=lt.size(i)
    return total

def addAirport(analyzer,airport):
    """
    Adiciona un aeropuerto al map aeropuertos y, como vertice, al grafo red
    Adiciona un aeropuerto al ordered map latlng 
    """
    # map aeropuertos y grafo red
    iata = airport['IATA']
    mp.put(analyzer['aeropuertos'],iata,airport)
    if not gr.containsVertex(analyzer['red'], iata):
            gr.insertVertex(analyzer['red'], iata)
    if not gr.containsVertex(analyzer['blue'], iata):
            gr.insertVertex(analyzer['blue'], iata)
    # ordered map latlng
    ordmap = analyzer['latlng']
    lat = float(airport['Latitude'])
    lng = float(airport['Longitude'])
    existlat = om.contains(ordmap,lat)
    if existlat:
        pareja = om.get(ordmap,lat)
        ordmap2 = me.getValue(pareja)
        existlng = om.contains(ordmap2,lng)
        if existlng:
            pareja2 = om.get(ordmap2,lng)
            lista = me.getValue(pareja2)
        else:
            lista = lt.newList()
        lt.addLast(lista,airport)
        om.put(ordmap2,lng,lista)
    else:
        ordmap2 = om.newMap()
        lista = lt.newList()
        lt.addLast(lista,airport)
        om.put(ordmap2,lng,lista)
        om.put(ordmap,lat,ordmap2)
    return analyzer

def addRoute(analyzer, route):
    """
    Adiciona una ruta como arco entre dos aeropuertos en el grafo red
    """
    origin = route['Departure']
    destination = route['Destination']
    distance = route['distance_km']
    gr.addEdge(analyzer['red'], origin, destination, float(distance))
    
    

def addCity(analyzer, city):
    """
    Adiciona una ciudad al map ciudades
    """
    # map ciudades
    name = city['city_ascii']
    ciudades = analyzer['ciudades']
    existcity = mp.contains(ciudades,name)
    if existcity:
        pareja = mp.get(ciudades,name)
        lista = me.getValue(pareja)
    else:
        lista = lt.newList()
    lt.addLast(lista,city)
    mp.put(ciudades,name,lista)
    # map ciudades_id
    id = city['id']
    ciudades_id = analyzer['ciudades_id']
    mp.put(ciudades_id,id,city)
    return analyzer

def addRouteND(analyzer, route):
    """
    Adiciona una ruta como arco entre dos aeropuertos en el grafo blue
    """
    origin = route['Departure']
    destination = route['Destination']
    distance = float(route['distance_km'])
    red = analyzer["red"]
    blue= analyzer["blue"]
    edge = gr.getEdge(red, destination, origin)
    edge2=None
    if(gr.containsVertex(blue,origin) and gr.containsVertex(blue,destination)):
        edge2 = gr.getEdge(blue, origin,destination)
    if edge is not None and edge2==None:
        gr.addEdge(analyzer["blue"], destination, origin, distance)
    return analyzer
def buscarAeropuerto(catalog,IATA):
    return me.getValue(mp.get(catalog["aeropuertos"],IATA))
#======================================================================
# Funciones de comparacion
#======================================================================

def ordenAscendenteD(a,b):
    if (a > b):
        return 0
    return -1
def ordenDescendente(a,b):
    return a[3]>b[3]
def compara_mapa(elemento1,elemento2):
    llave2=me.getKey(elemento2)
    if (elemento1 == llave2):
        return 0
    elif elemento1 > llave2:
        return 1
    else:
        return -1
#======================================================================
# Requerimientos
#======================================================================
# Requerimiento 1 
#======================================================================

def Requerimiento1(analyzer):
    grafo = analyzer['red']
    aeropuertos = gr.vertices(grafo) #n
    conexiones = lt.newList("ARRAY_LIST") 
    for aeropuerto in lt.iterator(aeropuertos):
        numero_entran = gr.indegree(grafo,aeropuerto)
        numero_salen=gr.outdegree(grafo,aeropuerto)
        entran_salen=numero_entran+numero_salen
        aeropuerto=buscarAeropuerto(analyzer,aeropuerto)
        guardar=(aeropuerto,numero_entran,numero_salen,entran_salen)
        if(entran_salen>0):
            lt.addLast(conexiones,guardar)
    conexiones=ms.sort(conexiones,ordenDescendente)
    return conexiones
# complejidad req 1: nlog(n)+8n


#======================================================================
# Requerimiento 2 
#======================================================================

def clusterAirports(analyzer, IATA1, IATA2):
    """
    Informa si dos aeropuertos estan fuertemente conectados.
    """
    analyzer["components"] = scc.KosarajuSCC(analyzer["red"])
    verificación = scc.stronglyConnected(analyzer["components"], IATA1, IATA2)
    total = scc.connectedComponents(analyzer["components"])
    return verificación, total

#======================================================================
# Requerimiento 3
#======================================================================

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    return c * r

def homonimos(lista_o,lista_d):
    print("-" * 50)
    print('Posibles ciudades de origen: ')
    for valor in lt.iterator(lista_o):
        print("ID: " + valor['id'] + " Nombre: " + valor["city_ascii"] + " Latitud: " + valor["lat"] + " Longitud: " + valor["lng"] 
                                                                    + " Pais: " + valor["country"] + " Subregion: " + valor['admin_name'])
    print("-" * 50)
    print('Posibles ciudades de destino: ')                                                        
    for valor in lt.iterator(lista_d):
        print("ID: " + valor['id'] + " Nombre: " + valor["city_ascii"] + " Latitud: " + valor["lat"] + " Longitud: " + valor["lng"] 
                                                                    + " Pais: " + valor["country"] + " Subregion: " + valor['admin_name'])
    print("-" * 50)                                                 
    id_o = input('Escoja la ciudad de origen deseada (ingrese el ID de la ciudad): ')
    id_d = input('Escoja la ciudad de destino deseada (ingrese el ID de la ciudad): ')
    ids = lt.newList()
    lt.addLast(ids,id_o)
    lt.addLast(ids,id_d)
    return ids

def Requerimiento3(analyzer,origen,destino):
    # hallamos las ciudades por su nombre
    ciudades = analyzer['ciudades']
    aeropuertos_ord = analyzer['latlng']
    origin = mp.get(ciudades,origen)
    lista_o = me.getValue(origin)
    destination = mp.get(ciudades,destino)
    lista_d = me.getValue(destination)
    # miramos si hay homonimos
    if (lt.size(lista_o)>1) or (lt.size(lista_d)>1):
        ids = homonimos(lista_o,lista_d)
        id_o = lt.removeFirst(ids)
        id_d = lt.removeFirst(ids)
        mapa_h = analyzer['ciudades_id']
        pareja_o = mp.get(mapa_h,id_o)
        pareja_d = mp.get(mapa_h,id_d)
        o = me.getValue(pareja_o)
        d = me.getValue(pareja_d)
    else:
        o = lt.removeLast(lista_o)
        d = lt.removeLast(lista_d)
    # calculamos aeropuertos cercanos a las ciudades de origen y destino
    aeropuertos_origen = lt.newList()
    aeropuertos_destino = lt.newList()
    o_lat = float(o['lat'])
    o_lng = float(o['lng'])
    d_lat = float(d['lat'])
    d_lng = float(d['lng'])
    # llenar lista aeropuertos_origen
    olat_1 = o_lat-1
    olat_2 = o_lat+1
    olng_1 = o_lng-1
    olng_2 = o_lng+1
    aer_o = om.values(aeropuertos_ord,olat_1,olat_2)
    for omap in lt.iterator(aer_o):
        listas = om.values(omap,olng_1,olng_2)
        for lista in lt.iterator(listas):
            tamano = lt.size(lista)
            i = 0
            while i<tamano:
                e = lt.removeLast(lista)
                lt.addLast(aeropuertos_origen,e)
                i += 1
    # llenar lista aeropuertos_destino
    dlat_1 = d_lat-1
    dlat_2 = d_lat+1
    dlng_1 = d_lng-1
    dlng_2 = d_lng+1
    aer_d = om.values(aeropuertos_ord,dlat_1,dlat_2)
    for omap2 in lt.iterator(aer_d):
        listas2 = om.values(omap2,dlng_1,dlng_2)
        for lista2 in lt.iterator(listas2):
            tamano = lt.size(lista2)
            j = 0
            while j<tamano:
                el = lt.removeLast(lista2)
                lt.addLast(aeropuertos_destino,el)
                j += 1
    # hallar el aeropuerto mas cercano al origen
    distancias_o = lt.newList()
    for a_o in lt.iterator(aeropuertos_origen):
        latitud = float(a_o['Latitude'])
        longitud = float(a_o['Longitude'])
        d = haversine(o_lng,o_lat,longitud,latitud)
        lt.addLast(distancias_o,d)
    ms.sort(distancias_o,ordenAscendenteD)
    dist_o = lt.firstElement(distancias_o)
    for a_o in lt.iterator(aeropuertos_origen):
        latitud_o = float(a_o['Latitude'])
        longitud_o = float(a_o['Longitude'])
        d_o = haversine(o_lng,o_lat,longitud_o,latitud_o)
        if d_o == dist_o:
            aero_o = a_o
            break
    # hallar el aeropuerto mas cercano al destino
    distancias_d = lt.newList()
    for a_d in lt.iterator(aeropuertos_destino):
        latitud_d = float(a_d['Latitude'])
        longitud_d = float(a_d['Longitude'])
        dista = haversine(d_lng,d_lat,longitud_d,latitud_d)
        lt.addLast(distancias_d,dista)
    ms.sort(distancias_d,ordenAscendenteD)
    dist_d = lt.firstElement(distancias_d)
    for a_d in lt.iterator(aeropuertos_destino):
        latitud_d = float(a_d['Latitude'])
        longitud_d = float(a_d['Longitude'])
        d_d = haversine(d_lng,d_lat,longitud_d,latitud_d)
        if d_d == dist_d:
            aero_d = a_d
            break
    # ver si existe ruta entre el aeropuerto de origen y el aeropuerto de destino mas cercanos a las ciudades
    # De ser necesario, iterar cambiando los aeropuertos por aeropuertos menos cercanos hasta que tal ruta exista
    k = 0
    while k < 1:
        grafo = analyzer['red']
        IATA_o = aero_o['IATA']
        IATA_d = aero_d['IATA']     
        search = djk.Dijkstra(grafo,IATA_o)
        existspath = djk.hasPathTo(search,IATA_d)
        if existspath:
            camino = djk.pathTo(search,IATA_d)
            latitud_o = float(aero_o['Latitude'])
            longitud_o = float(aero_o['Longitude'])
            latitud_d = float(aero_d['Latitude'])
            longitud_d = float(aero_d['Longitude'])
            dt_o = haversine(o_lng,o_lat,longitud_o,latitud_o)
            dt_d = haversine(d_lng,d_lat,longitud_d,latitud_d)
            k += 1
        else:
            o1 = lt.removeFirst(distancias_o)
            o2 = lt.firstElement(distancias_o)
            d1 =lt.removeFirst(distancias_d)
            d2 = lt.firstElement(distancias_d)
            if o2 <= d2:
                for a_o in lt.iterator(aeropuertos_origen):
                    latitud_o = float(a_o['Latitude'])
                    longitud_o = float(a_o['Longitude'])
                    d_o = haversine(o_lng,o_lat,longitud_o,latitud_o)
                    if d_o == o2:
                        aero_o = a_o
                        break
            if d2 < o2:
                for a_d in lt.iterator(aeropuertos_destino):
                    latitud_d = float(a_d['Latitude'])
                    longitud_d = float(a_d['Longitude'])
                    d_d = haversine(d_lng,d_lat,longitud_d,latitud_d)
                    if d_d == d2:
                        aero_d = a_d
                        break
    # estipular return
    resultado = lt.newList()
    lt.addLast(resultado,aero_o)
    lt.addLast(resultado,aero_d)
    lt.addLast(resultado,camino)
    lt.addLast(resultado,dt_o)
    lt.addLast(resultado,dt_d)
    return resultado

#======================================================================
# Requerimiento 4
#======================================================================

def aeroCercanoX(analyzer,ciudad):
    aeropuertos=analyzer["aeropuertos"]
    lat=float(ciudad["lat"])
    lon=float(ciudad["lng"])
    aeropuerto=None
    distancia=None
    for i in lt.iterator(mp.valueSet(aeropuertos)):
        if(distancia==None):
            aeropuerto=i
            distancia=haversine(lon,lat,float(i["Longitude"]),float(i["Latitude"]))
        else:
            dis=haversine(lon,lat,float(i["Longitude"]),float(i["Latitude"]))
            cuantos=gr.degree(analyzer["red"],i["IATA"])
            if(dis<=distancia and cuantos>0):
                aeropuerto=i
                distancia=dis
    return aeropuerto

def lifeMiles(analyzer, origen, millas):
    mst=prim.PrimMST(analyzer["blue"]) #n(cant vert)2
    arcos=prim.edgesMST(analyzer["blue"],mst)["mst"] #e(cant arc)
    aeroCercano=aeroCercanoX(analyzer,origen) #n
    grafo= gr.newGraph(datastructure='ADJ_LIST',directed=False,size=14000,comparefunction=compara_mapa)

    for i in lt.iterator(arcos): 
        VerticeA=i["vertexA"]
        VerticeB=i["vertexB"]
        if(not gr.containsVertex(grafo,VerticeA)):
            gr.insertVertex(grafo,VerticeA)
        if(not gr.containsVertex(grafo,VerticeB)):
            gr.insertVertex(grafo,VerticeB)
        gr.addEdge(grafo,VerticeA,VerticeB,i["weight"])
    #e
    km=millas*1.60
    DFS,info=DepthFirstSearch(grafo,aeroCercano["IATA"]) #n
    total=info["costo_Total"]*2
    dicts={"falto":0,"sobro":0,"igual":False}
    if(total >km):
        dicts["falto"]=total-km
    elif(total<km):
        dicts["sobro"]=km-total
    else:
        dicts["igual"]=True

    return info["arcos"],info["num_visitados"],dicts,total/2,aeroCercano

def DepthFirstSearch(graph, source):
    search = {'source': source,
              'visited': None}
    search['visited'] = mp.newMap(numelements=gr.numVertices(graph),
                                       maptype='PROBING',
                                       comparefunction=graph['comparefunction']
                                       )
    mp.put(search['visited'], source, {'marked': True, 'edgeTo': None})
    #Estructura para poder saber el límite de vertices que se pueden visitar con los kilómetros de límite.
    visitables={"arcos":lt.newList("ARRAY_LIST",cmpfunction=comparaLista),"costo_Total":0,"num_visitados":0}
    visitables["num_visitados"]+=1
    #Se agregan los vértices de inicio a la estructura
    dfsVertex(search, graph, source,visitables)
    return search,visitables
def dfsVertex(search, graph, vertex,visitables):
    #Se ordena la lista de adyacentes a vertex(inicial) para saber cuál es el más barato para comenzar.
    adjlst = gr.adjacents(graph, vertex)
    #Comienza DFS
    for w in lt.iterator(adjlst):
        visited = mp.get(search['visited'], w)
        if visited is None:
            mp.put(search['visited'],
                    w, {'marked': True, 'edgeTo': vertex})
            arco=gr.getEdge(graph,vertex,w)
            lt.addLast(visitables["arcos"],arco)
            visitables["costo_Total"]+=arco["weight"]
            visitables["num_visitados"]+=1
            dfsVertex(search, graph, w,visitables)

def compare_Vertex(ver1,ver2):
    return ver1[1]<=ver2[1]
def comparaLista(el1,el2):
    if (el1 == el2):
        return 0
    elif el1 > el2:
        return 1
    else:
        return -1
    
#======================================================================
# Requerimiento 5
#======================================================================

def Requerimiento5(analyzer,aeropuerto):
    grafo = analyzer['red']
    iatas = gr.adjacents(grafo,aeropuerto)
    aeropuertos = analyzer['aeropuertos']
    resultado = lt.newList()
    lt.addLast(resultado,lt.size(iatas))
    lista = lt.newList()
    for iata in lt.iterator(iatas):
        pareja = mp.get(aeropuertos,iata)
        aeropuerto = me.getValue(pareja)
        lt.addLast(lista,aeropuerto)
    lt.addLast(resultado,lista)
    return resultado