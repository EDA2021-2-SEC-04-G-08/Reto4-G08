"""
 * Copyright 2020, Departamento de sistemas y Computación, Universidad
 * de Los Andes
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

import prettytable
import config as cf
import sys
import controller
from DISClib.ADT import list as lt
from DISClib.ADT import graph as gr
from DISClib.ADT import map as mp
from DISClib.ADT import stack as st
from DISClib.DataStructures import mapentry as me
assert cf
import time
import threading
from prettytable import PrettyTable as pt

"""
La vista se encarga de la interacción con el usuario
Presenta el menu de opciones y por cada seleccion
se hace la solicitud al controlador para ejecutar la
operación solicitada
"""
#=================================================================================


def printMenu():
    print("*******************************************")
    print("Bienvenido")
    print("0- Crear catálogo vacio")
    print("1- Cargar información en el catálogo")
    print("2- Encontrar puntos de interconexión aérea ")
    print("3- Encontrar clústeres de tráfico aéreo ")
    print("4- Encontrar la ruta más corta entre ciudades ")
    print("5- Utilizar las millas de viajero ")
    print("6- Cuantificar el efecto de un aeropuerto cerrado ")
    print("7- Comparar con servicio WEB externo ")
    print("8- Salir")
    print("*******************************************")

def loadData(analyzer):
    """
    Carga los datos en la estructura de datos
    """
    return controller.loadData(analyzer)

analyzer = None


def printDataReq1(datos):
    size = lt.size(datos)
    if size>0:
        for dato in lt.iterator(datos):
            if dato is not None:
                print('IATA: ' + dato['IATA'] + ', Nombre: ' + dato['Name']
                     + ', Ciudad: ' + dato['City'] + ', Pais: ' + dato['Country']) 
    else:   
        print ("No se encontraron datos")

def printAeropuerto(dato):
    if dato is not None:
                print('IATA: ' + dato['IATA'] + ', Nombre: ' + dato['Name']
                     + ', Ciudad: ' + dato['City'] + ', Pais: ' + dato['Country'])

def printFirst(analyzer, indice):
    llaves = mp.keySet(analyzer[indice])
    p = lt.firstElement(llaves)
    primero = mp.get(analyzer[indice], p)
    valor = me.getValue(primero)
    if indice == "aeropuertos":
        print("Nombre: " + valor["Name"] + " Ciudad: " + valor["City"] + " País: " + valor["Country"] 
                            + " Latitud: " + valor["Latitude"] + " Longitud: " + valor["Longitude"])
    elif indice == "ciudades":
        valor = lt.removeLast(valor)
        print("Nombre: " + valor["city_ascii"] + " Latitud: " + valor["lat"] + " Longitud: " + valor["lng"] 
                                                                    + " Población: " + valor["population"])
                                                                    
def PrintCargaDatos(analyzer,primeroAero,ultimoAero,primeroCiudad,UltimoCiudad):
    print("-" * 50)
    print("Información Grafo Dirigido")
    print("-" * 50)
    print("Total de aeropuertos: " + str(gr.numVertices(analyzer['red'])))
    print("Total de rutas: " + str(gr.numEdges(analyzer['red'])))
    print("-" * 50)
    print("Información Grafo No Dirigido")
    print("-" * 50)
    print("Total de aeropuertos: " + str(gr.numVertices(analyzer['blue'])))
    print("Total de rutas: " + str(gr.numEdges(analyzer['blue'])))
    print("-" * 50)
    
    c=controller.contar_ciudades(analyzer["ciudades"])
    print("Total de ciudades: "+  str(c))
    
    print("-" * 50)
    print("Primer Aeropuerto Cargado y último aeropuerto cargado: ")
    tabla=pt()
    tabla.field_names=["Name","City","Country","IATA"]
    tabla.add_row([primeroAero["Name"],primeroAero["City"],primeroAero["Country"],primeroAero["IATA"]])
    tabla.add_row([ultimoAero["Name"],ultimoAero["City"],ultimoAero["Country"],ultimoAero["IATA"]])
    print(tabla)
    print("-" * 50)
    print("Primera y última Ciudad Cargada: ")
    tabla=pt()
    tabla.field_names=["city","lat","lng","country","admin_name","capital","population","id"]
    tabla.add_row([primeroCiudad["city"],primeroCiudad["lat"],primeroCiudad["lng"],primeroCiudad["country"],primeroCiudad["admin_name"],primeroCiudad["capital"],primeroCiudad["population"],primeroCiudad["id"]])
    tabla.add_row([UltimoCiudad["city"],UltimoCiudad["lat"],UltimoCiudad["lng"],UltimoCiudad["country"],UltimoCiudad["admin_name"],UltimoCiudad["capital"],UltimoCiudad["population"],UltimoCiudad["id"]])
    print(tabla)
    print("-" * 50 +"\n")

#=================================================================================

def cargaDatos(analyzer):
    return controller.loadData(analyzer)

def Requerimiento1(analyzer):
    aeropuertos = controller.Requerimiento1(analyzer)
    printDataReq1(aeropuertos)
    print("-" * 50)
    return None

def Requerimiento2(analyzer, Aero_1, Aero_2):
    verificación, total = controller.Requerimiento2(analyzer, Aero_1, Aero_2)
    print("-" * 50)
    print("Total de clusteres presentes: " + str(total))
    print("-" * 50)
    if verificación is True:
        print("Los aeropuertos " + Aero_1 + " y " + Aero_2 + " SI estan Conectados.")
    else:
        print("Los aeropuertos " + Aero_1 + " y " + Aero_2 + " NO estan Conectados.")

def Requerimiento3(analyzer,origen,destino):
    resultado = controller.Requerimiento3(analyzer,origen,destino)
    aero_o = lt.removeFirst(resultado)
    aero_d = lt.removeFirst(resultado)
    camino = lt.removeFirst(resultado)
    dt_o = lt.removeFirst(resultado)
    dt_d = lt.removeFirst(resultado)
    d_aerea = lt.newList()
    print("-" * 50)
    print('Aeropuerto de origen: ')
    printAeropuerto(aero_o)
    print('Aeropuerto de destino: ')
    printAeropuerto(aero_d)
    print("-" * 50)
    print('Ruta Aérea por segmentos: ')
    tamano = st.size(camino)
    i = 0
    while i < tamano:
        path = st.pop(camino)
        print('De: ' + str(path['vertexA']) + 'A: ' + str(path['vertexB']))
        print(str(path['weight']) + ' km')
        print('')
        lt.addLast(d_aerea,float(path['weight']))
        i += 1
    total_a = 0
    for distance in lt.iterator(d_aerea):
        total_a += distance
    print("-" * 50)
    print('Distancia total ruta: ' + str(int(dt_o+dt_d+total_a)))
    print('Distancia total aereo: ' + str(int(total_a)))
    print('Distancia ciudad-aeropuerto origen: ' + str(int(dt_o)))
    print('Distancia ciudad-aeropuerto destino: ' + str(int(dt_d)))
    return None

def Requerimiento4(analyzer):
    origen = input("Elija su ciudad de origen: ")
    millas = float(input("Ingrese sus millas disponibles: "))
    ciudades=me.getValue(mp.get(analyzer["ciudades"],origen))
    tabla=pt()
    tabla.field_names=["Elección","city","lat","lng","country","capital"]
    i=1
    for tupla in lt.iterator(ciudades):
        tabla.add_row([i,tupla["city"],tupla["lat"],tupla["lng"],tupla["country"],tupla["capital"]])
        i+=1
    print(tabla)
    elección=int(input("Digite el número de la ciudad:"))
    ciudad=lt.getElement(ciudades,elección)
    RamaMasLarga,numVertices,dicts,costo_total_mst,aero = controller.Requerimiento4(analyzer,ciudad, millas)
    print("El Aeropuerto de salida es:")
    tabla=pt()
    tabla.field_names=["Name","City","Country","IATA"]
    tabla.add_row([aero["Name"],aero["City"],aero["Country"],aero["IATA"]])
    print(tabla)
    print("El número de nodos conectados en el MST es:",numVertices)
    print("El costo total del MST partiendo desde:",ciudad["city"],"es:",costo_total_mst)
    print("La rama más larga consiste en :")
    tabla=pt()
    tabla.field_names=["Departure","Destination","distance_km"]
    for i in lt.iterator(RamaMasLarga):
         tabla.add_row([i["vertexA"],i["vertexB"],i["weight"]])
    print(tabla)
    if(dicts["falto"]>0):
        print("Faltó una cantidad de:",dicts["falto"]/1.60,"millas para conectar más ciudades")
    elif(dicts["sobro"]>0):
        print("Sobró una cantidad de:",dicts["sobro"]/1.60,"millas")
    else:
        print("Se alcanzaron todas los vértices")
    

def Requerimiento5(analyzer,aeropuerto):
    resultado = controller.Requerimiento5(analyzer,aeropuerto)
    num = lt.removeFirst(resultado)
    lista = lt.removeFirst(resultado)
    print('Numero de aeropuertos afectados: ' + str(num))
    print("-" * 50)
    print('Lista de aeropuertos afectados: ')
    printDataReq1(lista)
    return None

#================================================================================

def thread_cycle():
    while True:
        printMenu()
        inputs = input('Seleccione una opción para continuar\n')

        if int(inputs[0]) == 0:
            print("\nInicializando....")
            analyzer = controller.initCatalog()

        elif int(inputs[0]) == 1:
            print("\nCargando información de transporte aereo ....")
            start_time = time.process_time()
            analyzer,primeroAero,ultimoAero,primeroCiudad,UltimoCiudad=cargaDatos(analyzer)
            stop_time = time.process_time()
            elapsed_time_mseg = (stop_time - start_time)*1000
            print("Tiempo de ejecución: " + str(elapsed_time_mseg))
            PrintCargaDatos(analyzer,primeroAero,ultimoAero,primeroCiudad,UltimoCiudad)
        elif int(inputs[0]) == 2:
            start_time = time.process_time()
            Requerimiento1(analyzer)
            stop_time = time.process_time()
            elapsed_time_mseg = (stop_time - start_time)*1000
            print("Tiempo de ejecución: " + str(elapsed_time_mseg))

        elif int(inputs[0]) == 3:
            Aero_1 = input("Ingrese el Codigo IATA del primer Aeropuerto: ")
            Aero_2 = input("Ingrese el Codigo IATA del segundo Aeropuerto: ")
            start_time = time.process_time()
            Requerimiento2(analyzer, Aero_1, Aero_2)
            stop_time = time.process_time()
            elapsed_time_mseg = (stop_time - start_time)*1000
            print("Tiempo de ejecución: " + str(elapsed_time_mseg))
        
        elif int(inputs[0]) == 4:
            origen = input('Ciudad de origen: ')
            destino = input('Ciudad de destino: ' )
            start_time = time.process_time()
            Requerimiento3(analyzer,origen,destino)
            stop_time = time.process_time()
            elapsed_time_mseg = (stop_time - start_time)*1000
            print("Tiempo de ejecución: " + str(elapsed_time_mseg))
        
        elif int(inputs[0]) == 5:
            start_time = time.process_time()
            Requerimiento4(analyzer)
            stop_time = time.process_time()
            elapsed_time_mseg = (stop_time - start_time)*1000
            print("Tiempo de ejecución: " + str(elapsed_time_mseg))

        elif int(inputs[0]) == 6:
            aeropuerto = input('Ingrese el IATA del aeropuerto fuera de funcionamiento: ')
            start_time = time.process_time()
            Requerimiento5(analyzer,aeropuerto)
            stop_time = time.process_time()
            elapsed_time_mseg = (stop_time - start_time)*1000
            print("Tiempo de ejecución: " + str(elapsed_time_mseg))
        
            
        else:
            sys.exit(0)

if __name__ == "__main__":
    threading.stack_size(67108864)  # 64MB stack
    sys.setrecursionlimit(10000)
    thread = threading.Thread(target=thread_cycle)
    thread.start()