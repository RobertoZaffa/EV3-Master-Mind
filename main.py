#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import random
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

# Initialize the motors 
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
x_motor = Motor(Port.D)
pick_motor = Motor(Port.A)

touch_sensor = TouchSensor(Port.S4)
color_sensor = ColorSensor(Port.S1)


def pick(): #fa scendere la pinza per prendere un piolo e poi risale
    # Fa salire il pick up
    pick_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=70)
    pick_motor.reset_angle(0)
    # Fa scendere il pick up fino al peg e lo prende. Rallenta prima di prenderlo
    pick_motor.run_target(200, -200)
    pick_motor.run_target(100, -340)
    #wait_sensor_pressed()
    # Fa salire il pick up
    pick_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=70)
    pick_motor.reset_angle(0)
    
def release(): #fa scendere la pinza per posare i pioli e poi risale
    pick_motor.run_target(200, -220)
    # Fa salire il pick up
    pick_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=70)

def wait_sensor_pressed(): #Aspetta fino a quando il touch sensor non è pigiato
  while not touch_sensor.pressed():
    wait(10)
  wait (100)
  pressed=True
  return pressed

def reset_position(): #porta il carrello in fondo a SX e resetta la posizione
  # Azzera posizione pick motor -  run_until_stalled(speed, then=Stop.COAST, duty_limit=None)
  pick_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
  left_motor.stop()  # or brake()
  right_motor.stop()  # or brake()
  # reduce the motors power
  left_motor.control.limits(actuation=15)
  right_motor.control.limits(actuation=15)
  # run_time(speed, time, then=Stop.HOLD, wait=True)
  left_motor.run_time(300,15000, then=Stop.COAST, wait=False)
  right_motor.run_time(300,15000, then=Stop.COAST, wait=False)
  # Wait until it is stalled
  while not left_motor.control.stalled() and not right_motor.control.stalled():
    wait(10)
  # Stop the motor (you can also choose brake or hold here)
  left_motor.stop()
  right_motor.stop()
  left_motor.reset_angle(0)
  right_motor.reset_angle(0)
  # restore the motors power
  left_motor.control.limits(actuation=100)
  right_motor.control.limits(actuation=100)
  # Sposta il carrello tutto a sx
  x_motor.run_until_stalled(1200, then=Stop.COAST, duty_limit=50)
  x_motor.reset_angle(0)

def res_angle(): #resetta la posizione dei motori delle coordinate x,y
  left_motor.reset_angle(0)
  right_motor.reset_angle(0)
  x_motor.reset_angle(0)

def go(px,py): #Sposta il carrello al peg da giocare
  # Sposta il carrello al peg da giocare
  x_motor.run_target(400,-px)
  x_motor.stop()
  # run_target(speed, target_angle, then=Stop.HOLD, wait=True)
  left_motor.run_target(350, -py, then=Stop.HOLD, wait=False)
  right_motor.run_target(350, -py, then=Stop.HOLD, wait=True)
  left_motor.stop()  # or brake()
  right_motor.stop()  # or brake()

def color_to_play(color): #Trova il primo piolo di colore color disponibile. Poi chiama go_pick()
  #next_peg contiene la posizione del primo piolo disponibile per ogni colore
  next_peg[color]=next_peg[color]+1
  #print ("next_peg ",next_peg)
  px=next_peg[color]
  py=color
  go_pick(px,py)
  
def go_pick(px,py): #Converte coordinate px,py in coordinate per EV3. poi chiama go(px,py) e pick()
  px=916+(px-1)*148
  py=139+(py-1)*151
  go(px,py)
  pick()

def go_rel(px,py): #Converte coordinate px,py in coordinate per EV3. Poi chiama go(px,py) e release()
  px=3+(px-1)*148
  py=85+(py-1)*180
  go(px,py)
  release()

def tuning0(): #Sequenza di presa/rilascio pioli usando go_pick() e go_rel()
  
  
  go_pick(10,1)
  go_rel(4,1)

  go_pick(10,1)
  go_rel(3,1)

  go_pick(10,1)
  go_rel(2,1)
  
  go_pick(4,1)
  go_rel(1,2)
  
  go_pick(5,1)
  go_rel(4,2)
  
  go_pick(6,1)
  go_rel(3,2)
  
  go_pick(7,1)
  go_rel(1,6)
  
  go_pick(8,1)
  go_rel(2,6)

  go_pick(9,1)
  go_rel(4,6)

  go_pick(10,1)
  go_rel(3,6)
  
def tuning1(): #Sequenza di presa/rilascio pioli usando color_to_play() e go_rel()
  color_to_play(1)
  go_rel(1,1)

  color_to_play(2)
  go_rel(2,1)

  color_to_play(1)
  go_rel(3,1)

  color_to_play(3)
  go_rel(4,1)
 
  color_to_play(1)
  go_rel(1,2)

  color_to_play(2)
  go_rel(2,2)

  color_to_play(1)
  go_rel(3,2)

  color_to_play(3)
  go_rel(4,2)
   
def scan_keycode(): #Scan with color sensor the key code 
  #ev3.speaker.set_volume(10)
  global bianchi
  global neri
  bianchi=0
  neri=0
  px0=1435
  py0=1650
  delta=220
  px=[px0, px0+delta, px0+delta, px0  ]
  py=[py0, py0, py0+delta, py0+delta]
  pick_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=70)
  pick_motor.reset_angle(0)
  for i in range(0, 4):
      go(px[i],py[i])
      pick_motor.run_target(300, -310)
      wait(100)
      print (color_sensor.color())
      if str(color_sensor.color()) == "Color.WHITE":
        bianchi=bianchi+1
      if str(color_sensor.color()) == "Color.BLACK":
        neri=neri+1
      #wait_sensor_pressed()
  pick_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=70)
  pick_motor.reset_angle(0)
  ch=[]
  ch.append(bianchi)
  ch.append(neri)
  return ch

def init_db(): #inizializza db e db_ft uguali - crea anche db_bc
    for d in range(1, 7):
      for e in range(1, 7):
        for b in range(1, 7):
          for c in range(1, 7):
            temp = [d, e, b, c, True, 0, 0, 0, 0, 0, 0, 0] #inizializza i cp
            temp[4 + d] += 1 #inizializza 6 campi, uno per colore con il numero di volte che il colore è presente
            temp[4 + e] += 1
            temp[4 + b] += 1
            temp[4 + c] += 1
            if temp[5]<3 and temp[6]<3 and temp[7]<3 and temp[8]<3 and temp[9]<3 and temp[10]<3:
                db_bc.append(temp) #metti solo i cp che hanno meno di tre colori riprtuti in db_bc
            db.append(temp)
            db_ft.append(temp)

def play_code(board_row,cp):
  for column in range (1,5):
    color_to_play(cp[column-1]) #prende il peg di cp in posizione da 0 a 3
    go_rel(column,board_row)
    
def popola_ft(ft,db,cp,ch): #crea un nuovo db_ft con i cp possibili
    db_ft=[]
    for i in range (0, 1296):
        ch_test=test(cp,db[i]) #confronta il cp con tutti i cp di db
        if ch==ch_test and db[i][4]==True: #se ch è = a ch inserito da human e il cp non è già stato eliminato, aggiungi a db_ft
            db_ft.append(db[i])
        else:
            db[i][4]=False #se il cp non è un ft, elimina da db
    return db_ft

def test(cp,c_ft): #confronta due combinazioni e ritorna il ch
    ch_test=[0,0]
    bn=0 #bianchi + neri
    neri=0
    for i in range(5, 11): #prendi il minor numero di volte che un colore è presente
        if cp[i] < c_ft[i]:
            bn=bn+cp[i]
        else:
            bn=bn+c_ft[i]
    for i in range(0,4): #calcola il numeri di neri
        if cp[i]==c_ft[i]:
            neri += 1
    ch_test[0]=bn-neri #calcola i bianchi per differenza
    ch_test[1]=neri
    return ch_test

def popola_bc(ft,db,db_ft,db_bc):
    min = 9999
    db_bc=[]
    for i in range (0,ft): #ripeti per tutti gli ft
        ch_hit=[]
        for a in range (0,5):
            ch_hit.append([0,0,0,0,0])
        for j in range (0,ft):
            ch_test=test(db_ft[i], db_ft[j]) #trova ch tra i cp e i possibili cs
            ch_hit[ch_test[0]][ch_test[1]] +=1 #incrementa le ricorrenze di ch uguali
        max=0
        for x in range(0,5): #trova il valore massimo in ch_hit
            for y in range(0,5):
              if ch_hit[x][y] > max:
                 max = ch_hit[x][y]
        db_ft[i][11] = max #assegna max al cp
        if max < min: # assegna a min il minore tra i max
            min = max
    flag=False #flag diveta true se c'è almeno un ft in db_bc
    for i in range (0,ft):
        if db_ft[i][11] == min:
            db_bc.append(db_ft[i])
    return db_bc #crea il db dei bc dalla prima giocata e le successive

#reset_position()
#res_angle()
#ch=scan_keycode()
#print (ch)
#tuning0()
#tuning1()
#go(100,500)
#go(0,0)
#cp = random.randrange(0, 2) 


while True:
    print ("Inserisci il Codice Segreto e premi il pulsante")
    wait_sensor_pressed()
    #say scegli e nascondi il CS e poi pigia il bottone
    db = []
    db_ft = []
    db_bc = []
    init_db()
    ft = 1296 #per la prima giocata gli ft sono tutti i possibili codici
    bc = 1170 #per la prima giocata, i bc sono la somma dei tipi 1234, 1123, 1122
    board_row=0
    next_peg=[0,0,0,0,0,0,0,] #mantiene il primo peg disponibile per ogni colore
    reset_position()
    res_angle()
    while board_row < 11: #al massimo 10 codici tentativo
        board_row +=1
        #print ("board_row", board_row)
        if bc>1: 
          cp = db_bc[random.randrange(0, bc)] #gioca un cp random tra tutti i bc
        else: #random.randrange(0,1) è ok con python ma da errore con Micropython
          cp = db_bc[bc-1] #se c'è un solo bc, cp punta alla entry 0
        if board_row==1: #gioca come primo tentativo 3 5 2 1
          cp=db_bc[523]
        print ("cp", cp)
        play_code (board_row,cp)
        if ft==1: #se il cp giocato è l'ultimo, il cs è indovinato
            print ("codice indovinato")
            break #partita terminata
        print ("inserisci il Codice Chiave e premi il pulsante")    
        pressed=wait_sensor_pressed()
        ch=scan_keycode() #human inserisce il ch
        print ("ch ", ch)
        if ch[1]==4: #se inserisce 4 neri il cs è indovinato
            print ("codice indovinato")
            break
        db_ft=popola_ft(ft,db,cp,ch)
        ft = len(db_ft)
        print ("ft ", ft)
        if ft<15:
           for temp in db_ft:
              print(temp)
        if ft==0:
            # say pernacchia
            print ("Almeno un codice chiave è errato")
            break
        if ft<110: # chiama popola_bc solo se gli ft non sono troppi. Altrimenti è troppo lento
            print ("Elaborazione in corso...")
            db_bc=popola_bc(ft,db,db_ft,db_bc)
        else:
            db_bc=db_ft.copy()
        bc = len(db_bc)
        #print ("bc ", bc)
    print ("Premi il pulsante per un'altra partita")
    wait_sensor_pressed()
    go(0,0)


