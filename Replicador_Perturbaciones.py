import openpyxl, serial
import numpy as np
from time import time
from time import sleep


# Configuración Base de Datos
client = InfluxDBClient(host='localhost', port=8086, username='admin', password='robotica')
client.create_database('Lighting')

mn1= 'Lamp1'
mn2= 'Lamp2'
mn3= 'Lamp3'
mn4= 'Lamp4'
mn5= 'Lamp5'
mn6= 'Lamp6'
mn7= 'Lamp7'
mn8= 'Lamp8'
mn9= 'Lamp9'
mn10= 'Window1'
mn11= 'Window2'

data=[]

# Cargar constantes de lámparas
doc = openpyxl.load_workbook('Constantes2.xlsx')
hoja = doc.get_sheet_by_name('Hoja1')
kij=np.zeros((9,9))
for f in range(9):
    g=0
    for c in 'ABCDEFGHI':
        celda=c+str(f+1)
        kij[f][g]=float(hoja[celda].value)
        g=g+1
# Cargar constantes perturbaciones 
doc1 = openpyxl.load_workbook('ConsPert3.xlsx')
hoja1 = doc1.get_sheet_by_name('Hoja1')
kip=np.zeros((9,2))
for f in range(9):
    g=0
    for c in 'AB':
        celda=c+str(f+1)
        kip[f][g]=float(hoja1[celda].value)
        g=g+1

# Vectores para guardar información del comportamiento durante la ejecución
tiempo=[]
salida1=[]
salida2=[]
salida3=[]
salida4=[]
salida5=[]
salida6=[]
salida7=[]
salida8=[]
salida9=[]

pwm1=[]
pwm2=[]
pwm3=[]
pwm4=[]
pwm5=[]
pwm6=[]
pwm7=[]
pwm8=[]
pwm9=[]
pwm10=[]

x1=[]
x2=[]
x3=[]
x4=[]
x5=[]
x6=[]
x7=[]
x8=[]
x9=[]

aux=0

Ts=1#Muestreo

P=5000 # Potencia disponoible
f=np.zeros((10,1))# Vector de funciones fitness
s=np.zeros((9,1))# Vector valor de iluminancia en sensores
y=np.zeros((9,1))
ya=np.zeros((9,1))
# Inicializar Potencia
xa=100*np.ones((10,1))
xa[9][0]=4100
x=100*np.ones((10,1))
x[9][0]=4100# Zona Ficticia
ua=np.zeros((10,1))
r=150 # Set Point general
R=r*np.ones((9,1))
B=1000#800
alpha=0.1#500
beta=0.44#0.51


pert=np.zeros((2,1))# Vector influencia externa
for j in range(601): # Tiempo de ejecución
    # if j==301: # Cambio de Set POint
    #     r=250
    #     R=r*np.ones((9,1))
    start=time()
    # Conexión Tarjetas (info sensores)
    arduino1=serial.Serial('/dev/rfcomm0',9600)
    arduino2=serial.Serial('/dev/rfcomm7',9600)
    arduino3=serial.Serial('/dev/rfcomm2',9600)
    arduino4=serial.Serial('/dev/rfcomm3',9600)
    arduino5=serial.Serial('/dev/rfcomm4',9600)
    arduino6=serial.Serial('/dev/rfcomm5',9600)
    arduino7=serial.Serial('/dev/rfcomm6',9600)
    

    s1_1=arduino1.readline()
    s2_1=arduino2.readline()
    s3_1=arduino3.readline()
    s4_1=arduino4.readline()
    s5_1=arduino5.readline()
    s6_1=arduino6.readline()
    s7_1=arduino7.readline()

    # Lectura Sensores
    
    while s1_1.find('.')==-1:
        s1_1=arduino1.readline()
    
    while s2_1.find('.')==-1:
        s2_1=arduino2.readline()
    
    while s3_1.find('.')==-1:
        s3_1=arduino3.readline()
    
    while s4_1.find('.')==-1:
        s4_1=arduino4.readline()

    while s5_1.find('.')==-1:
        s5_1=arduino5.readline()
    
    while s6_1.find('.')==-1:
        s6_1=arduino6.readline()
    
    while s7_1.find('.')==-1:
        s7_1=arduino7.readline()

    arduino1.close()
    arduino2.close()
    arduino3.close()
    arduino4.close()
    arduino5.close()
    arduino6.close()
    arduino7.close()

     # Lectura por texto Bluetooth 2

    f8=open ('med8.txt','r')
    s8_1=f8.readline()
    f8.close()
    while s8_1.find('.')==-1:
        f8=open ('med8.txt','r')
        s8_1=f8.readline()
        f8.close()


    f9=open ('med9.txt','r')
    s9_1=f9.readline()
    f9.close()
    while s9_1.find('.')==-1:
        f9=open ('med9.txt','r')
        s9_1=f9.readline()
        f9.close()    

    f10=open ('med10.txt','r')
    s10_1=f10.readline()
    f10.close()
    while s10_1.find('.')==-1:
        f10=open ('med10.txt','r')
        s10_1=f10.readline()
        f10.close()

    f11=open ('med11.txt','r')
    s11_1=f11.readline()
    f11.close()
    while s11_1.find('.')==-1:
        f11=open ('med11.txt','r')
        s11_1=f11.readline()
        f11.close()     

    #Conversión valores sensores lámparas
    s[0][0]=float(s1_1)
    s[1][0]=float(s2_1)
    s[2][0]=float(s3_1)
    s[3][0]=float(s4_1)
    s[4][0]=float(s5_1)
    s[5][0]=float(s6_1)
    s[6][0]=float(s7_1)
    s[7][0]=float(s8_1)
    s[8][0]=float(s9_1)

    #Lectura sensores ventanas(perturbaciones luz externa)
    pert[0][0]=float(s10_1)
    pert[1][0]=float(s11_1)

    y=np.dot(kij,s)+np.dot(kip,pert)

#Replicador
    for i in range(0,9):
        f[i][0]=x[i][0]*(B+R[i][0]-y[i][0])#Fitness de cada zona
    suma=0
    for l in range(0,9):
        suma=f[l][0]+suma
    f[9][0]=x[9][0]*B#Fitness de zona ficticia
    suma=suma+f[9][0]
    Fb=suma/P#Fitness promedio

    for p in range(0,9):
        x[p][0]=round(xa[p][0]*(beta*(alpha+(B+R[p][0]-ya[p][0]))/(alpha+Fb)),2)#Potencias

    x[9][0]=round(xa[9][0]*(beta*(alpha+B)/(alpha+Fb)),2)#Potencia ficticia
    
    xa[:,:]=x
    ya[:,:]=y
    
    #Ajuste por Software cuando la potencia es cercana a cero
    for q in range(0,9):
        if (x[q][0]<10):
            aux2=10-x[q][0]
            x[q][0]=10
            x[9][0]=x[9][0]-aux2

    #Saturación de señales
    for q in range(0,9):
        ua[q][0]=x[q][0]+10
        if ua[q][0]>100:
            ua[q][0]=100
    ua[9][0]=x[9][0]    


    # Escribir archivos de texto con la señal de control
    fa=open('exp1.txt','w')
    fa.write(str(ua[0][0]))
    fa.close()
    fa=open('exp2.txt','w')
    fa.write(str(ua[1][0]))
    fa.close()
    fa=open('exp3.txt','w')
    fa.write(str(ua[2][0]))
    fa.close()
    fa=open('exp4.txt','w')
    fa.write(str(ua[3][0]))
    fa.close()
    fa=open('exp5.txt','w')
    fa.write(str(ua[4][0]))
    fa.close()
    fa=open('exp6.txt','w')
    fa.write(str(ua[5][0]))
    fa.close()
    fa=open('exp7.txt','w')
    fa.write(str(ua[6][0]))
    fa.close()
    fa=open('exp8.txt','w')
    fa.write(str(ua[7][0]))
    fa.close()
    fa=open('exp9.txt','w')
    fa.write(str(ua[8][0]))
    fa.close()

    tiempo.extend([aux])
    aux=aux+Ts
    salida1.extend([y[0][0]])
    salida2.extend([y[1][0]])
    salida3.extend([y[2][0]])
    salida4.extend([y[3][0]])
    salida5.extend([y[4][0]])
    salida6.extend([y[5][0]])
    salida7.extend([y[6][0]])
    salida8.extend([y[7][0]])
    salida9.extend([y[8][0]])

    pwm1.extend([ua[0][0]])
    pwm2.extend([ua[1][0]])
    pwm3.extend([ua[2][0]])
    pwm4.extend([ua[3][0]])
    pwm5.extend([ua[4][0]])
    pwm6.extend([ua[5][0]])
    pwm7.extend([ua[6][0]])
    pwm8.extend([ua[7][0]])
    pwm9.extend([ua[8][0]])
    pwm10.extend([ua[9][0]])

    x1.extend([x[0][0]])
    x2.extend([x[1][0]])
    x3.extend([x[2][0]])
    x4.extend([x[3][0]])
    x5.extend([x[4][0]])
    x6.extend([x[5][0]])
    x7.extend([x[6][0]])
    x8.extend([x[7][0]])
    x9.extend([x[8][0]])


    #Guardar Información Base de Datos
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn1,
                    SALIDA=y[0][0],
                    PWM=u[0][0],
                    timestamp=int(time()*1000)))

    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn2,
                    SALIDA=y[1][0],
                    PWM=u[1][0],
                    timestamp=int(time()*1000)))

    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn3,
                    SALIDA=y[2][0],
                    PWM=u[2][0],
                    timestamp=int(time()*1000)))
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn4,
                    SALIDA=y[3][0],
                    PWM=u[3][0],
                    timestamp=int(time()*1000)))
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn5,
                    SALIDA=y[4][0],
                    PWM=u[4][0],
                    timestamp=int(time()*1000)))
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn6,
                    SALIDA=y[5][0],
                    PWM=u[5][0],
                    timestamp=int(time()*1000)))
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn7,
                    SALIDA=y[6][0],
                    PWM=u[6][0],
                    timestamp=int(time()*1000)))
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn8,
                    SALIDA=y[7][0],
                    PWM=u[7][0],
                    timestamp=int(time()*1000)))
    data.append("{measurement} SALIDA={SALIDA},PWM={PWM} {timestamp}"
            .format(measurement=mn9,
                    SALIDA=y[8][0],
                    PWM=u[8][0],
                    timestamp=int(time()*1000)))
    client.write_points(data, database='Lighting', time_precision='ms', batch_size=10000, protocol='line')

    print(s)
    print(x)
    print(y)
    print(j)

    fin=time()-start
    Pause=Ts-fin
    if Pause<0:
        Pause=0
    sleep(Pause)
##########################################################################
#Exportar archivos de texto para conocer comportamiento durante el tiempo de ejecución
size=len(salida1)
for k in range(size):
    t=str(tiempo[k])

    L1=str(salida1[k])
    L2=str(salida2[k])
    L3=str(salida3[k])
    L4=str(salida4[k])
    L5=str(salida5[k])
    L6=str(salida6[k])
    L7=str(salida7[k])
    L8=str(salida8[k])
    L9=str(salida9[k])

    c1=str(pwm1[k])
    c2=str(pwm2[k])
    c3=str(pwm3[k])
    c4=str(pwm4[k])
    c5=str(pwm5[k])
    c6=str(pwm6[k])
    c7=str(pwm7[k])
    c8=str(pwm8[k])
    c9=str(pwm9[k])
    c10=str(pwm10[k])

    k1=str(x1[k])
    k2=str(x2[k])
    k3=str(x3[k])
    k4=str(x4[k])
    k5=str(x5[k])
    k6=str(x6[k])
    k7=str(x7[k])
    k8=str(x8[k])
    k9=str(x9[k])

    if k==0:
        time = open('tiempo.txt','w')
        time.writelines(t)
        time.close()

        file1 = open('salida1.txt','w')
        file2 = open('salida2.txt','w')
        file3 = open('salida3.txt','w')
        file4 = open('salida4.txt','w')
        file5 = open('salida5.txt','w')
        file6 = open('salida6.txt','w')
        file7 = open('salida7.txt','w')
        file8 = open('salida8.txt','w')
        file9 = open('salida9.txt','w') 
        file1.writelines(L1)
        file2.writelines(L2)
        file3.writelines(L3)
        file4.writelines(L4)
        file5.writelines(L5)
        file6.writelines(L6)
        file7.writelines(L7)
        file8.writelines(L8)
        file9.writelines(L9)
        file1.close()
        file2.close()
        file3.close()
        file4.close()
        file5.close()
        file6.close()
        file7.close()
        file8.close()
        file9.close()

        xile1 = open('pwm1.txt','w')
        xile2 = open('pwm2.txt','w')
        xile3 = open('pwm3.txt','w')
        xile4 = open('pwm4.txt','w')
        xile5 = open('pwm5.txt','w')
        xile6 = open('pwm6.txt','w')
        xile7 = open('pwm7.txt','w')
        xile8 = open('pwm8.txt','w')
        xile9 = open('pwm9.txt','w')
        xile10 = open('pwm10.txt','w')
        xile1.writelines(c1)
        xile2.writelines(c2)
        xile3.writelines(c3)
        xile4.writelines(c4)
        xile5.writelines(c5)
        xile6.writelines(c6)
        xile7.writelines(c7)
        xile8.writelines(c8)
        xile9.writelines(c9)
        xile10.writelines(c10)
        xile1.close()
        xile2.close()
        xile3.close()
        xile4.close()
        xile5.close()
        xile6.close()
        xile7.close()
        xile8.close()
        xile9.close()
        xile10.close()

        zile1 = open('x1.txt','w')
        zile2 = open('x2.txt','w')
        zile3 = open('x3.txt','w')
        zile4 = open('x4.txt','w')
        zile5 = open('x5.txt','w')
        zile6 = open('x6.txt','w')
        zile7 = open('x7.txt','w')
        zile8 = open('x8.txt','w')
        zile9 = open('x9.txt','w')
      
        zile1.writelines(k1)
        zile2.writelines(k2)
        zile3.writelines(k3)
        zile4.writelines(k4)
        zile5.writelines(k5)
        zile6.writelines(k6)
        zile7.writelines(k7)
        zile8.writelines(k8)
        zile9.writelines(k9)
        
        zile1.close()
        zile2.close()
        zile3.close()
        zile4.close()
        zile5.close()
        zile6.close()
        zile7.close()
        zile8.close()
        zile9.close()
    else:

        time = open('tiempo.txt','a')
        time.writelines('\n'+t)
        time.close()

        file1 = open('salida1.txt','a')
        file2 = open('salida2.txt','a')
        file3 = open('salida3.txt','a')
        file4 = open('salida4.txt','a')
        file5 = open('salida5.txt','a')
        file6 = open('salida6.txt','a')
        file7 = open('salida7.txt','a')
        file8 = open('salida8.txt','a')
        file9 = open('salida9.txt','a') 
        file1.writelines('\n'+L1)
        file2.writelines('\n'+L2)
        file3.writelines('\n'+L3)
        file4.writelines('\n'+L4)
        file5.writelines('\n'+L5)
        file6.writelines('\n'+L6)
        file7.writelines('\n'+L7)
        file8.writelines('\n'+L8)
        file9.writelines('\n'+L9)
        file1.close()
        file2.close()
        file3.close()
        file4.close()
        file5.close()
        file6.close()
        file7.close()
        file8.close()
        file9.close()

        xile1 = open('pwm1.txt','a')
        xile2 = open('pwm2.txt','a')
        xile3 = open('pwm3.txt','a')
        xile4 = open('pwm4.txt','a')
        xile5 = open('pwm5.txt','a')
        xile6 = open('pwm6.txt','a')
        xile7 = open('pwm7.txt','a')
        xile8 = open('pwm8.txt','a')
        xile9 = open('pwm9.txt','a')
        xile10 = open('pwm10.txt','a')
        xile1.writelines('\n'+c1)
        xile2.writelines('\n'+c2)
        xile3.writelines('\n'+c3)
        xile4.writelines('\n'+c4)
        xile5.writelines('\n'+c5)
        xile6.writelines('\n'+c6)
        xile7.writelines('\n'+c7)
        xile8.writelines('\n'+c8)
        xile9.writelines('\n'+c9)
        xile10.writelines('\n'+c10)
        xile1.close()
        xile2.close()
        xile3.close()
        xile4.close()
        xile5.close()
        xile6.close()
        xile7.close()
        xile8.close()
        xile9.close()
        xile10.close()

        zile1 = open('x1.txt','a')
        zile2 = open('x2.txt','a')
        zile3 = open('x3.txt','a')
        zile4 = open('x4.txt','a')
        zile5 = open('x5.txt','a')
        zile6 = open('x6.txt','a')
        zile7 = open('x7.txt','a')
        zile8 = open('x8.txt','a')
        zile9 = open('x9.txt','a')
      
        zile1.writelines('\n'+k1)
        zile2.writelines('\n'+k2)
        zile3.writelines('\n'+k3)
        zile4.writelines('\n'+k4)
        zile5.writelines('\n'+k5)
        zile6.writelines('\n'+k6)
        zile7.writelines('\n'+k7)
        zile8.writelines('\n'+k8)
        zile9.writelines('\n'+k9)
        
        zile1.close()
        zile2.close()
        zile3.close()
        zile4.close()
        zile5.close()
        zile6.close()
        zile7.close()
        zile8.close()
        zile9.close()