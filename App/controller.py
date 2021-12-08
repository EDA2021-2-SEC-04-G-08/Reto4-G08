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
 """
import config as cf
import model
import csv


"""
El controlador se encarga de mediar entre la vista y el modelo.
"""

#======================================
# Inicialización del Catálogo
#======================================

def initCatalog():
    """
    Llama la funcion de inicializacion del catalogo del modelo.
    """
    analyzer = model.newAnalyzer()
    return analyzer

#==================================
# Funciones para la carga de datos
#==================================

def loadData(analyzer):
    """
    Carga los datos de los archivos y cargar los datos en la
    estructura de datos
    """
    loadAirports(analyzer)
    loadRoutes(analyzer)
    loadCities(analyzer)
    loadRoutesND(analyzer)
    return analyzer

def loadAirports(catalog):
    """
    Carga los aeropuertos.
    """
    booksfile = cf.data_dir + 'Skylines/airports-utf8-small.csv'
    input_file = csv.DictReader(open(booksfile, encoding='utf-8'))
    for airport in input_file:
        model.addAirport(catalog, airport)
    
def loadRoutes(catalog):
    """
    Carga las rutas.
    """
    booksfile = cf.data_dir + 'Skylines/routes-utf8-small.csv'
    input_file = csv.DictReader(open(booksfile, encoding='utf-8'))
    for route in input_file:
        model.addRoute(catalog, route)

def loadCities(analyzer):
    """
    Carga las ciudades
    """
    booksfile = cf.data_dir +'Skylines/worldcities.csv'
    input_file = csv.DictReader(open(booksfile, encoding='utf-8'))
    for city in input_file:
        model.addCity(analyzer, city)

def loadRoutesND(catalog):
    """
    Carga las rutas.
    """
    booksfile = cf.data_dir + 'Skylines/routes-utf8-small.csv'
    input_file = csv.DictReader(open(booksfile, encoding='utf-8'))
    for route in input_file:
        model.addRouteND(catalog, route)

def loadGreen(analyzer, servicesfile):
    """
    Carga las rutas bien hecho.
    """
    servicesfile = cf.data_dir + servicesfile
    input_file = csv.DictReader(open(servicesfile, encoding="utf-8"),
                                delimiter=",")
    lastflight = None
    for flight in input_file:
        if lastflight is not None:
            sameservice = lastflight['Airline'] == flight['Airline']
            samedirection = lastflight['Destination'] == flight['Destination']
            samebusStop = lastflight['Departure'] == flight['Departure']
            if sameservice and samedirection and not samebusStop:
                model.addAirportConnection(analyzer, lastflight, flight)
        lastflight = flight
    return analyzer


#========================================================
# Requerimientos
#========================================================

def Requerimiento1(analyzer):
    return model.Requerimiento1(analyzer)

def Requerimiento2(analyzer, IATA1, IATA2):
    return model.clusterAirports(analyzer, IATA1, IATA2)

def Requerimiento3(analyzer,origen,destino):
    return model.Requerimiento3(analyzer,origen,destino)

def Requerimiento4(analyzer, origen, millas):
    return model.lifeMiles(analyzer, origen, millas)

def Requerimiento5(analyzer,aeropuerto):
    return model.Requerimiento5(analyzer,aeropuerto)