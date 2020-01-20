import openpyxl, serial
import numpy as np
from numpy.linalg import inv
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

################Funcion QPhild##################################################
# Cálculo multiplicadores de Lagrange
def QPhild(H,f,A_cons,b):
    n1=len(A_cons)
    eta=-1*np.dot(inv(H),f)
    kk=0
    for i in range(0,n1):
        if (np.dot(A_cons[i,:],eta)>b[i]):
            kk=kk+1
        else:
            kk=kk+0
    if (kk==0):
        return eta
    P=np.dot(A_cons,np.dot(inv(H),A_cons.transpose()))
    d=np.dot(A_cons,np.dot(inv(H),f))+b
    n=len(d)
    m=len(d[0])
    x_ini=np.zeros((n,m))
    lambda1=x_ini    
    al=10
    for km in range(0,38):
        lambda_p=lambda1
        for i in range(0,n):
            w=np.dot(P[i,:],lambda1)-np.dot(P[i,i],lambda1[i,0])
            w=w+d[i,0]
            #print(w)
            la=-w/P[i,i]
            lambda1[i,0]=np.max([0,la])
        al=np.dot((lambda1-lambda_p).transpose(),lambda1-lambda_p)
        if (al<10e-8):
            break
    eta=np.dot(inv(-1*H),f)+np.dot(np.dot(inv(-1*H),A_cons.transpose()),lambda1)
    return eta

############################################################################################
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

aux=0

Ts=1#Muestreo

#Espacio de estados discretizado
Ad=0.0821*np.eye(9)
Ad[0][0]=0.2865
Ad[8][8]=0.0067
Bd=np.zeros((9,9))
Bd[0][0]=13.3349
Bd[1][1]=34.9535
Bd[2][2]=21.9596
Bd[3][3]=35.9369
Bd[4][4]=16.4213
Bd[5][5]=13.2463
Bd[6][6]=20.2617
Bd[7][7]=22.1621
Bd[8][8]=29.1992

Cd=kij
Dd=np.zeros((9,9))

m1=len(Cd)
n1=len(Bd)

# Cálculo de Matrices PHI y F
n_in=len(Bd[0])
An=np.eye(m1+n1)
An[0:n1,0:n1]=Ad
An[n1:n1+m1,0:n1]=np.dot(Cd,Ad)
Bn=np.zeros((n1+m1,n_in))
Bn[0:n1,:]=Bd
Bn[n1:n1+m1,:]=np.dot(Cd,Bd)
Cn=np.zeros((m1,n1+m1))
Cn[:,n1:n1+m1]=np.eye(m1)
n=len(Cn)
Np=2
Nc=1
F=np.zeros((Np*n,2*n_in))
F[0:n,:]=np.dot(Cn,An)
for i in range(0,Np-1):
    A1=An
    j=1
    while j<=i+1:
        A1=np.dot(A1,An)
        j=j+1
    F[n:n+n1,:]=np.dot(Cn,A1)
    n=n+n1
n=len(Cn)
m=len(Cn[0])
n2=n
Phi=np.zeros((Np*n,Nc*n))
Phi[0:n,0:n]=np.dot(Cn,Bn)
for i in range(0,Np-1):
    j=1
    A1=An
    while j<i+1:
        A1=np.dot(A1,An)
        j=j+1
    Prod=np.dot(Cn,A1)
    Phi[n:n+n1,0:n2]=np.dot(Prod,Bn)
    Phi[n:n+n1,n2:len(Phi[0])]=Phi[n-n1:n,0:n2*(Nc-1)]
    n=n+n1
Phi_Phi=np.dot(Phi.transpose(),Phi)
Phi_F=np.dot(Phi.transpose(),F)
Phi_R=Phi_F[:,len(Phi_F[0])-n2:len(Phi_F[0])]
xm=np.zeros((9,1))
xm_old=np.zeros((9,1))
Xf=np.zeros((2*n2,1))
u=np.zeros((9,1))
ua=np.zeros((9,1))
y=np.zeros((9,1))
R=1000*np.eye(len(Phi_Phi))
r=150
Ref=r*np.ones((9,1))
Ref1=r*np.ones((9,1))
d=np.zeros((9,1))
d[0][0]=18.6895
d[1][0]=38.0792
d[2][0]=23.9234
d[3][0]=39.1506
d[4][0]=17.8898
d[5][0]=14.4309
d[6][0]=22.0736 
d[7][0]=24.1440
d[8][0]=29.3973
pert=np.zeros((2,1))

#########Restricciones####################
n_in=9
aux=9

A_cons=np.zeros((2*Nc*n_in,n_in*Nc))

#sleep(5)
A_cons[0:n_in,0:n_in]=np.eye(n_in)
#print(len(A_cons[0]))

for i in range(1,Nc):
    A_cons[aux:aux+n_in,n_in:len(A_cons[0])]=A_cons[aux-n_in:aux,0:n_in*(Nc-1)]
    A_cons[aux:aux+n_in,0:n_in]=np.eye(n_in)
    aux=aux+n_in

A_cons[aux:aux+n_in,0:n_in]=-1*np.eye(n_in)



A_cons[aux:aux+n_in,n_in:len(A_cons[0])]=np.zeros((n_in,n_in*(Nc-1)))
for i in range(0,Nc):
    A_cons[aux:aux+n_in,n_in:len(A_cons[0])]=A_cons[aux-n_in:aux,0:n_in*(Nc-1)]
    A_cons[aux:aux+n_in,0:n_in]=-1*np.eye(n_in)
    aux=aux+n_in

umin=0
umax=100
DeltaU=np.zeros((Nc*9,1))
b=np.zeros((2*n_in*Nc,1))
###########################################
aux0=0
for j in range(601): # Tiempo de Ejecución
    """ if j==301:  # Cambio de Set Point
        r=250
        Ref1=r*np.ones((9,1)) """
    start=time()
    
    # Conexión Tarjetas (info sensores)
    arduino1=serial.Serial('/dev/rfcomm0',9600)
    arduino2=serial.Serial('/dev/rfcomm7',9600)
    arduino3=serial.Serial('/dev/rfcomm2',9600)
    arduino4=serial.Serial('/dev/rfcomm3',9600)
    arduino5=serial.Serial('/dev/rfcomm4',9600)
    arduino6=serial.Serial('/dev/rfcomm5',9600)
    arduino7=serial.Serial('/dev/rfcomm6',9600)

    # Lectura Sensores
    s1_1=arduino1.readline()
    s2_1=arduino2.readline()
    s3_1=arduino3.readline()
    s4_1=arduino4.readline()
    s5_1=arduino5.readline()
    s6_1=arduino6.readline()
    s7_1=arduino7.readline() 
    
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

    print(s1_1)
    print(s2_1)
    print(s3_1)
    print(s4_1)
    print(s5_1)
    print(s6_1)
    print(s7_1)

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
    

    # Cálculo de la señal de control con restricciones 
    #DeltaU=np.dot(inv(Phi_Phi+R),np.dot(Phi_R,Ref)-np.dot(Phi_F,Xf))
    H=2*Phi_Phi+R
    f=-2*(np.dot(Phi_R,Ref)-np.dot(Phi_F,Xf))
    aux=0
    for i in range(0,Nc):
        b[aux:aux+n_in,:]=umax*np.ones((9,1))-u
        aux=aux+n_in
    for i in range(0,Nc):
        b[aux:aux+n_in,:]=umin*np.ones((9,1))+u
        aux=aux+n_in
    #print(A_cons)
    DeltaU=QPhild(H,f,A_cons,b)
    deltau=DeltaU[0:n2,:]
    u=u+deltau
    #print(u)
    xm_old[:,:]=xm

    xm[0][0]=float(s1_1)
    xm[1][0]=float(s2_1)
    xm[2][0]=float(s3_1)
    xm[3][0]=float(s4_1)
    xm[4][0]=float(s5_1)
    xm[5][0]=float(s6_1)
    xm[6][0]=float(s7_1)
    xm[7][0]=float(s8_1)
    xm[8][0]=float(s9_1)

    pert[0][0]=float(s10_1)
    pert[1][0]=float(s11_1)

    y=np.dot(kij,xm)

    yr=y+np.dot(kip,pert)

    Ref=Ref1-np.dot(kip,pert)

    Xf[0:n2,:]=xm-xm_old
    Xf[n2:2*n2,:]=y
    
    print(deltau)
    print(u)
    print(yr)
    print(j)

    #Solución Explícita con parámetros d
    ua[0][0]=-6.5559/(2*0.1215)+(((6.5559)**2-4*0.1215*(-1.6376-d[0][0]*u[0][0]))**0.5)/(2*0.1215)
    ua[1][0]=-7.1012/(2*0.3183)+(((7.1012)**2-4*0.3183*(-85.205-d[1][0]*u[1][0]))**0.5)/(2*0.3183)
    ua[2][0]=-4.7038/(2*0.1976)+(((4.7038)**2-4*0.1976*(-54.036-d[2][0]*u[2][0]))**0.5)/(2*0.1976)
    ua[3][0]=-7.1792/(2*0.3281)+(((7.1792)**2-4*0.3281*(-83.856-d[3][0]*u[3][0]))**0.5)/(2*0.3281)
    ua[4][0]=-4.2495/(2*0.1408)+(((4.2495)**2-4*0.1408*(-43.973-d[4][0]*u[4][0]))**0.5)/(2*0.1408)
    ua[5][0]=-3.8355/(2*0.1094)+(((3.8355)**2-4*0.1094*(-34.462-d[5][0]*u[5][0]))**0.5)/(2*0.1094)
    ua[6][0]=-3.7761/(2*0.1879)+(((3.7761)**2-4*0.1879*(-49.251-d[6][0]*u[6][0]))**0.5)/(2*0.1879)
    ua[7][0]=-5.3571/(2*0.1933)+(((5.3571)**2-4*0.1933*(-54.307-d[7][0]*u[7][0]))**0.5)/(2*0.1933)
    ua[8][0]=-5.8920/(2*0.2418)+(((5.8920)**2-4*0.2418*(-67.466-d[8][0]*u[8][0]))**0.5)/(2*0.2418)

    
    """ for q in range(9):
        ua[q][0]=round(ua[q][0],2)

        if ua[q][0]>100:
            ua[q][0]=100
        
        if ua[q][0]<0:
            ua[q][0]=0.01 """

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

    tiempo.extend([aux0])
    aux0=aux0+Ts
    salida1.extend([yr[0][0]])
    salida2.extend([yr[1][0]])
    salida3.extend([yr[2][0]])
    salida4.extend([yr[3][0]])
    salida5.extend([yr[4][0]])
    salida6.extend([yr[5][0]])
    salida7.extend([yr[6][0]])
    salida8.extend([yr[7][0]])
    salida9.extend([yr[8][0]])

    pwm1.extend([u[0][0]])
    pwm2.extend([u[1][0]])
    pwm3.extend([u[2][0]])
    pwm4.extend([u[3][0]])
    pwm5.extend([u[4][0]])
    pwm6.extend([u[5][0]])
    pwm7.extend([u[6][0]])
    pwm8.extend([u[7][0]])
    pwm9.extend([u[8][0]])

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
    
    fin=time()-start
    Pause=Ts-fin
    if Pause<0:
        Pause=0    
    sleep(Pause) 
##################################################################
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
    #c10=str(pwm10[k])


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
        
        xile1.writelines(c1)
        xile2.writelines(c2)
        xile3.writelines(c3)
        xile4.writelines(c4)
        xile5.writelines(c5)
        xile6.writelines(c6)
        xile7.writelines(c7)
        xile8.writelines(c8)
        xile9.writelines(c9)
        
        xile1.close()
        xile2.close()
        xile3.close()
        xile4.close()
        xile5.close()
        xile6.close()
        xile7.close()
        xile8.close()
        xile9.close()
        
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
        
        xile1.writelines('\n'+c1)
        xile2.writelines('\n'+c2)
        xile3.writelines('\n'+c3)
        xile4.writelines('\n'+c4)
        xile5.writelines('\n'+c5)
        xile6.writelines('\n'+c6)
        xile7.writelines('\n'+c7)
        xile8.writelines('\n'+c8)
        xile9.writelines('\n'+c9)
        
        xile1.close()
        xile2.close()
        xile3.close()
        xile4.close()
        xile5.close()
        xile6.close()
        xile7.close()
        xile8.close()
        xile9.close()
        