% UNIVERSIDAD INDUSTRIAL DE SANTANDER
% INSTRUMENTACION ELECTRONICA
% PROYECTO:
	% DISENO DE UN SISTEMA DE SENSORES INALAMBRICO DRONE-EQUIPABLE
% INTERFAZ GRAFICA: 
	% CORRER EL CODIGO CON EL ARDUINO CONECTADO MEDIANTE USB
% INTEGRANTES:
	% Fernandez Laude - 2122264,
	% Pico Jean Alejandro - 2112259,
	% Salazar Isail - 2122307,
	% Rovira Maria Fernanda - 2122256,
	% Vasquez Paula - 2120542.
	
arduino = serial('COM4','BaudRate',9600); 

fopen(arduino); 

N = 50;

b1 = zeros(1,N);
b2 = b1;
b3 = b1;
b4 = b1;
b5 = b1;
b6 = b1;
b7 = b1;
b8 = b1;
b9 = b1;

bind = 1:N;

for i = 1:50
fscanf(arduino)
pause(100e-3)
end    

schar_i = fscanf(arduino); 
chars = strsplit(schar_i);

if numel(chars)>7
   T1_i = str2double(chars(1));
   p_mb_i= str2double(chars(2));
   alt_msnm_i = str2double(chars(3));
   hum_perc_i = str2double(chars(4));
   T2_i = str2double(chars(5)); 
   xa_i = str2double(chars(6)); 
   ya_i = str2double(chars(7));
   za_i = str2double(chars(8));
   co2ppm_i = str2double(chars(9));
end
if numel(chars)<=7
   T1_i = str2double(chars(1));
   p_mb_i= str2double(chars(2));
   alt_msnm_i = str2double(chars(3));
   hum_perc_i = str2double(chars(4));
   T2_i = str2double(chars(5)); 
   co2ppm_i = str2double(chars(6));
end


k = 1;
while (true)
schar = fscanf(arduino); 
chars = strsplit(schar);

if numel(chars)>7
   T1 = str2double(chars(1));
   p_mb = str2double(chars(2));
   alt_msnm = str2double(chars(3));
   hum_perc = str2double(chars(4));
   T2 = str2double(chars(5)); 
   xa = str2double(chars(6)); 
   ya = str2double(chars(7));
   za = str2double(chars(8));
   co2ppm = str2double(chars(9));
   
    b1(k) = T1;
    b2(k) = p_mb;
    b3(k) = alt_msnm;
    b4(k) = hum_perc;
    b5(k) = T2;
    b6(k) = xa;
    b7(k) = ya;
    b8(k) = za;
    b9(k) = co2ppm;
    
   k = k + 1;
if k == N+1
    k = 1;
    T1_i = b1(N);
    p_mb_i = b2(N); 
    alt_msnm_i = b3(N);
    hum_perc_i = b4(N);
    T2_i = b5(N);
    xa_i = b6(N);
    ya_i = b7(N);
    za_i = b8(N);
    co2ppm_i = b9(N);
  
    b1 = zeros(1,N);
    b2 = b1;
    b3 = b1;
    b4 = b1;
    b5 = b1;
    b6 = b1;
    b7 = b1;
    b8 = b1;
    b9 = b1;
end 
subplot(2,3,1),
plot(bind,b1,bind,b5)
title('Temperatura (ºC)')
axis([1 N T1_i-10 T1_i+10])
grid on
subplot(2,3,2),
plot(bind,b2,'linewidth',2,'Color','r');
title('Presión Barométrica (mb)')
axis([1 N p_mb_i-100 p_mb_i+100])
grid on
subplot(2,3,3),
plot(bind,b3,'linewidth',2,'Color','g');
title('Altura (m.s.n.m.)')
axis([1 N p_mb_i-100 p_mb_i+100])
grid on
subplot(2,3,4),
plot(bind,b4,'linewidth',2,'Color','m');
title('Porcentaje de Humedad')
axis([1 N hum_perc_i-10 hum_perc_i+10])
grid on
subplot(2,3,5),
%plot(bind,b6,bind,b7,bind,b8)
title('Posicion X,Y,Z')
[cyx,cyy,cyz]=cylinder(.4);
[conx,cony,conz]=cylinder([1,0]);
[spx,spy,spz]=sphere(50);
cyz = cyz+1;
conz = conz+2;
figure(2), surf(cyx,cyy,cyz), hold on
figure(2), surf(conx,cony,conz,'FaceColor','r')
figure(2), surf(spx,spy,spz), hold off
axis([-2.5 2.5 -2.5 2.5 -3.5 3.5]);
ylabel('y')
xlabel('x')
zlabel('z')
%axis([1 N xa_i-50 xa_i+50])
grid on
subplot(2,3,6),
plot(bind,b9,'linewidth',2,'Color','g');
title('Concentración de CO2 (ppm)')
axis([1 N co2ppm_i-500 co2ppm_i+500])
grid on
end


if numel(chars)<=7
   T1 = str2double(chars(1));
   p_mb = str2double(chars(2));
   alt_msnm = str2double(chars(3));
   hum_perc = str2double(chars(4));
   T2 = str2double(chars(5)); 
   co2ppm = str2double(chars(6));
   
    b1(k) = T1;
    b2(k) = p_mb;
    b3(k) = alt_msnm;
    b4(k) = hum_perc;
    b5(k) = T2;
    b9(k) = co2ppm;
    
   k = k + 1;
if k == N+1
    k = 1;
    T1_i = b1(N);
    p_mb_i = b2(N); 
    alt_msnm_i = b3(N);
    hum_perc_i = b4(N);
    T2_i = b5(N);
    xa_i = b6(N);
    ya_i = b7(N);
    za_i = b8(N);
    co2ppm_i = b9(N);
  
    b1 = zeros(1,N);
    b2 = b1;
    b3 = b1;
    b4 = b1;
    b5 = b1;
    b6 = b1;
    b7 = b1;
    b8 = b1;
    b9 = b1;
end
subplot(2,3,1),
plot(bind,b1,bind,b5)
title('Temperatura (ºC)')
axis([1 N T1_i-10 T1_i+10])
grid on
subplot(2,3,2),
plot(bind,b2,'linewidth',2,'Color','r');
title('Presión Barométrica (mb)')
axis([1 N p_mb_i-100 p_mb_i+100])
grid on
subplot(2,3,3),
plot(bind,b3,'linewidth',2,'Color','g');
title('Altura (m.s.n.m.)')
axis([1 N p_mb_i-100 p_mb_i+100])
grid on
subplot(2,3,4),
plot(bind,b4,'linewidth',2,'Color','m');
title('Porcentaje de Humedad')
axis([1 N hum_perc_i-10 hum_perc_i+10])
grid on
subplot(2,3,5),
%plot(bind,b6,bind,b7,bind,b8)
title('Posicion X,Y,Z')
[cyx,cyy,cyz]=cylinder(.4);
[conx,cony,conz]=cylinder([1,0]);
[spx,spy,spz]=sphere(50);
cyz = cyz+1;
conz = conz+2;
figure(2), surf(cyx,cyy,cyz), hold on
figure(2), surf(conx,cony,conz,'FaceColor','r')
figure(2), surf(spx,spy,spz), hold off
axis([-2.5 2.5 -2.5 2.5 -3.5 3.5]);
ylabel('y')
xlabel('x')
zlabel('z')
%axis([1 N xa_i-50 xa_i+50])
grid on
subplot(2,3,6),
plot(bind,b9,'linewidth',2,'Color','g');
title('Concentración de CO2 (ppm)')
axis([1 N co2ppm_i-500 co2ppm_i+500])
grid on
end

pause(250e-3);
end


fclose(arduino);




[cyx,cyy,cyz]=cylinder(.4);
[conx,cony,conz]=cylinder([1,0]);
[spx,spy,spz]=sphere(50);
cyz = cyz+1;
conz = conz+2;
figure(2), surf(cyx,cyy,cyz), hold on
figure(2), surf(conx,cony,conz,'FaceColor','r')
figure(2), surf(spx,spy,spz), hold off
axis([-2.5 2.5 -2.5 2.5 -3.5 3.5]);















