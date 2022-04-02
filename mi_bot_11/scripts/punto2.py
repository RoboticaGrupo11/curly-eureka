#!/usr/bin/env python3

import imp
from turtle import color, position
import rospkg
import rospy
import tkinter as tk
import threading as th

from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

from mi_bot_11.srv import save_route_srv, replay_tour_srv
from PIL import ImageGrab
from tkinter import filedialog


PATH_BACKGROUND = rospkg.RosPack().get_path('mi_bot_11')+'/resources/Background3.png'

PATH_RESOURCES= rospkg.RosPack().get_path('mi_bot_11')+'/resources/'
        
def drawLine( canvas, x1, y1, x2, y2):
    canvas.create_line(x1, y1, x2, y2)

class Punto2:
    #Parametros de tamaño de la interfaz
    sizeDifWin = 50
    sizeImg = 550
    sizeWin = sizeImg+sizeDifWin
    # Ruta donde se guardará la imagén del recorrido, por defecto resources
    pathImage2Save = PATH_RESOURCES
    fileName=''
    # Boolean y ruta que corresponden al guardado del recorrido
    boolSaveRec = False
    path2SaveRec = ''
    #Ruta de la cual se cargará el recorrido y nombre del archivo
    path2LoadRec = ''

    #Constructor de la clase
    def __init__(self):
        #Publishers
        self.popUp2SaveRec() #Se corre en primer lugar la ventana emergente que permitirá preguntar si se desea salvar el recorrido

        # Se despliega la interfaz
        self.win = tk.Tk()
        self.canvas = tk.Canvas()
        self.xInit=self.sizeImg/2
        self.yInit=self.sizeImg/2
        self.xAct=self.sizeImg/2
        self.yAct=self.sizeImg/2

        #Se declara el servicio para guardado de ruta
        self.saveRouteServer = rospy.Service('save_route', save_route_srv, self.callback_save_route_srv)

        #Se declara el servicioProxy para cargar la ruta
        self.loadRouteClient = rospy.ServiceProxy('/turtle_bot_player/replay_tour_srv', replay_tour_srv)

    #   ------------------------------------------ INTERFACE ------------------------------------------
    def launchInterface(self):
        
        self.win.title("Interfaz de trayectoria")
        self.win.geometry(str(self.sizeWin)+"x"+str(self.sizeWin + self.sizeDifWin*1))

        
        self.canvas = tk.Canvas(self.win, height=self.sizeImg, width=self.sizeImg)
        self.canvas.place(x=25, y=25, anchor = "nw")
        bg = tk.PhotoImage(file=PATH_BACKGROUND)
        image_id = self.canvas.create_image(0,0,image=bg, anchor="nw")
        self.canvas.grid(column=0,row=0)

        frame1 = tk.Frame(self.win, bg='white')
        frame1.grid(column=0,row=1)
        entry1 = tk.Entry(frame1, font=("ComicSans"))
        entry1.insert(tk.END,"name")
        entry1.grid(column=1, row=0)

        ### Boton capturar imagen
        #Función que define el manejo de evento del botón capturar imagen. Llama a la función getShot 
        def shotImage():
            self.getShot(self.canvas, entry1.get())

        btn1 = tk.Button(frame1, text="Capturar imagen", font=("ComicSans"), command = shotImage)
        btn1.grid(column=0, row=0)


        ### Boton cambiar path de defecto para guardar imagen
        #Define el manejo de evento para cambiar el path por defecto donde se guardará la imagen.
        def changeImagePath():
            self.pathImage2Save = filedialog.askdirectory()
        btn12= tk.Button(frame1, text="Change path", font=("ComicSans"), command = changeImagePath)
        btn12.grid(column=2, row=0)

        #### ------------------------------------------ Punto 4 - LOAD  ------------------------------------------
        label3 = tk.Label(frame1,text="")
        label3.grid(column=1, row=2)

        # Define el manejo del evento del botón cargar un recorrido. Pide el path y el nombre
        # Además, realiza el proceso de conexión con el servicio de cargar el recorrido 
        def loadRec( ):
            self.path2LoadRec = filedialog.askdirectory()
            txt="Se cargo el recorrido"
            label3.config(text=txt)
            self.PATH = self.loadRouteClient.call(self.path2LoadRec)

        btn3 = tk.Button(frame1, text="Cargar un recorrido"  , font=("ComicSans"), command = loadRec)
        btn3.grid(column=0, row=2)


        ## ------------------------------ DESPLIEGUE DEL SUSCRIPTOR A TURTLE_BOT_POSITION ------------------------------
        ## Inicia el thread de suscripcion al topico.
        th_node = th.Thread(target=self.suscribePosition)
        th_node.start()

        # Inicia el loop de la interfaz
        self.win.mainloop()


    ## ------------------------------ FUNCIONES Y CALLBACKS ------------------------------



    #-------------- Funciones de interfaz gráfica  - Punto 2


    # FUNCTION que maneja la creación de la imagen del recorrido actual y la guarda en el path establecido
    def getShot(self,widget,fileName):
        x=self.win.winfo_rootx()+widget.winfo_x()
        y=self.win.winfo_rooty()+widget.winfo_y()
        x1=x+widget.winfo_width()
        y1=y+widget.winfo_height()
        ImageGrab.grab().crop((x,y,x1,y1)).save(self.pathImage2Save+"/"+ fileName+".png")

    ##-------------ROS--------------
    #Permite obtener la información de posicion del turtle bot a partir del topico. +
    # Llama al metodo drawLine para dibujar el recorrido actual
    def callback_drawLine(self, odom_data):
        position =  odom_data.pose.pose.position     
        posx = position.x*self.sizeImg/5 + self.sizeImg/2
        posy = position.y*-self.sizeImg/5 + self.sizeImg/2
        #str2Print="x: "+str(posx)+"  y: "+str(posy)
        #rospy.loginfo(rospy.get_caller_id()+"I_heard----- %s" , str2Print)
        self.win.after(1,  drawLine(self.canvas,self.xInit,self.yInit,posx,posy))
        self.xInit=posx
        self.yInit=posy

    # Define la suscripción al topico de position del odometry y asigna el callback
    def suscribePosition(self):
        rospy.Subscriber('odom', Odometry, self.callback_drawLine)
        rospy.spin()


    ## --------------- Funciones de Save recorrido - Punto 3

    # MANEJADOR VENTANA EMERGENTE AL INICIAR NODO
    def popUp2SaveRec(self):
        self.temproot = tk.Tk()
        self.temproot.title("¿Desea guardar esté recorrido?")
        label = tk.Label(self.temproot, text="Escriba el nombre")
        label.grid(row=0,column=0)

        entryTmp = tk.Entry(self.temproot, font=("ComicSans"))
        entryTmp.insert(tk.END, "name" )
        entryTmp.grid(column=1, row=0)

        #MAneja el evento de presionar SI al botón ded guardar
        def saveRec():
            self.boolSaveRec = True
            self.path2SaveRec = filedialog.askdirectory()
            
            self.path2SaveRec += "/"+entryTmp.get()
            self.temproot.destroy()
        
        B1 = tk.Button(self.temproot, text="SI", command = saveRec)
        B1.grid(row=1, column = 0)
        B2 = tk.Button(self.temproot, text="NO", command = self.temproot.destroy)
        B2.grid(row=1, column = 1)
        self.temproot.mainloop()

    ## --- CALLBACK para el servicio de guardar ruta, retorna la ruta en la cual se guardará 
    # el recorrido de ser el caso
    def callback_save_route_srv(self, req):
        return self.path2SaveRec


# Main del programa, lanza el despliegue de la interfaz en el hilo principal.
if __name__ == '__main__':
    rospy.init_node('turtle_bot_interface', anonymous=True)
    try:   
            punto2 = Punto2()
            punto2.launchInterface()
    except rospy.ROSInterruptException:
            pass
