clear all;
clc;

%---------------------------------------------------------------------%
%                              Datos globales                         %
%---------------------------------------------------------------------%

Fs = 44100;     %Frecuencia de muestreo
duracion = 3;   %Duracion de la captura
grosor = 4;
xMax = 300;

%---------------------------------------------------------------------%
%      A) Realice una captura de audio de música y voz y guarde       %
%---------------------------------------------------------------------%

%disp('Ingrese el nombre entre comilla simple');        %Muestra un mensaje en pantalla

%nombreC = input('nombre de la captura de audio : ');   %Lee el nombre ingresado por teclado
%input('Pulse Enter para comenzar la captura');
disp('Ha comenzado la grabacion shhh...');
nombreC = 'captura';                                    %Nombre predeterminado de la captura
senalC = wavrecord(duracion*Fs,Fs);                     %Capturando el audio
Vm = 1;
wavwrite(senalC, Fs, nombreC);                          %Guardando el archivo
disp('El audio ha sido guardado exitosamente.');        %Muestra un mensaje en pantalla   
input('Pulse Enter para reproducir');
sound(senalC,Fs);


%---------------------------------------------------------------------%
%                 C) Grafique la señal de entrada                     %
%---------------------------------------------------------------------%

figure('Name','Audio capturado','NumberTitle','off');

plot(senalC,'g');
title('\bfAudio capturado');     %Coloca el titulo a la gráfica
%axis([0,]);
grid on;
%---------------------------------------------------------------------%
%        D) y E) Determine y grafique el espectro de frecuencia       %
%---------------------------------------------------------------------%

figure('Name','Espectro de frecuencias del Audio capturado','NumberTitle','off');

MuSenalC = length(senalC);                           %Muestras de la señal (Tonos sumados)
FuSenalC = fft(senalC, MuSenalC);                     %Uso de la transformada rapida de fourier para encontrar los componentes de frecuencia de una señal
FrSenalC = linspace(-Fs/2, Fs/2, MuSenalC);           %Genera n puntos para la grafica
AmpSenalC = fftshift( abs( FuSenalC ) );              %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 

stem(FrSenalC, AmpSenalC, 'g','linewidth',grosor/4);  %Grafica del audio capturado con color verde
title('\bfEspectro de Frecuencia de la señal: Audio Capturado');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');                  %Coloca una etiqueta horizontal (eje x)
%ylabel('Amplitud (V)');                    %Coloca una etiqueta vertical (eje y)
axis([0, 1000, 10, 1000]);                  %Muestra solo la parte de la funcion en los intervalos especificados


%---------------------------------------------------------------------%
%                 F) y G) Module y grafique la señal                  %
%---------------------------------------------------------------------%


figure('Name','Modulacion en Am de Señal: Audio Capturado','NumberTitle','off');

Vc = 1;                             %Amplitud de la portadora
Fc = 15000;                     %Frecuencia de la portadora debe ser al menos 2 veces mayor a la frecuencia de la señal de informacion
Tc = 1/Fc;                          %Periodo de la portadora
Wc = 2*pi*Fc;                       %Frecuencia angular
% Vac = Vc*sin(Wc*t);                 %Funcion de la señal portadora 

% Vmax = Vc + Vmt;
% Vmin = Vc - Vmt;
% 
% Eusf = (Vmax-Vmin)/4;
% Eisf = Eusf;
% cp = Eusf + Eisf; %Cambio Pico de amplitud
% 
%Vam =(1+Vt).*Vac;

Vam = ammod(senalC, Fc, Fs);         %Modulacion de la señal
subplot(2,1,1);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 1
plot(senalC,'r','linewidth',grosor/4); %Grafica las funciones Vt con color rojo
%axis([0, 50000, -Vm, Vm]);         %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Informacion');              %Coloca una etiqueta vertical (eje y)

% subplot(3,1,2);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 2
% plot(Vac,'g','linewidth',grosor);%Grafica las funciones Vac con color verde
% %axis([0, n*Tt, -Vc, Vc]);           %Muestra solo la parte de la funcion en los intervalos especificados
% ylabel('Portadora');                %Coloca una etiqueta vertical (eje y)

subplot(2,1,2);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 3
plot(Vam,'b','linewidth',grosor/4);%Grafica la funcion Vam con color azul
%axis([0, 50000, -Vm, Vm]);         %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Modulada');                 %Coloca una etiqueta vertical (eje y)


%---------------------------------------------------------------------%
%   H) e I)  Determine espectro de frecuencia (Modulada) y grafique   %
%---------------------------------------------------------------------%

figure('Name','Espectro de frecuencias de la Señal Modulada: Captura de audio','NumberTitle','off');

NMod = length(Vam);                  %Muestras
FourierMod = fft(Vam, NMod);         %Espectro mediante fft
LinMod =linspace(-Fs/2, Fs/2, NMod);    %Genera n puntos para la grafica
AmpMod = fftshift( abs( FourierMod ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
stem(LinMod, AmpMod, 'b');              %Grafica la funcion Vam con color azul
axis([1e4, 2.5e4, 0, 400]);          %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal modulada: Audio Capturado');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');              %Coloca una etiqueta horizontal (eje x)


%---------------------------------------------------------------------%
%           J) y K)  Demodule, muestre y compare la señal             %
%---------------------------------------------------------------------%

figure('Name','Demodulacion de la señal: Captura de audio','NumberTitle','off');

senalCDemod = amdemod(Vam,Fc,Fs);   %Demodulacion de la señal amdemod(Vam,Fc,Fs)
plot(senalCDemod,'r','linewidth',grosor); %Grafica la señal demodulada con color rojo
hold on;
plot(senalC,'black--','linewidth',grosor);       %Grafica la señal original con color azul
legend('Señal Original (Audio)','Señal (Audio) Demodulada'); %Coloca una leyenda a la grafica
ylabel('Amplitud (V)');                                    %Coloca una etiqueta vertical (eje y)
title('\bfDemodulacion de la señal: Audio Capturado');       %Coloca el titulo a la gráfica
axis([10000, 12000, -0.8, 0.6]);                    %Muestra solo la parte de la funcion en los intervalos especificados

%---------------------------------------------------------------------%
%  L) y M)  Determine espectro de frecuencia (Deodulada) y grafique   %
%---------------------------------------------------------------------%

figure('Name','Espectro de frecuencias de la Señal Dmodulada: Captura de audio','NumberTitle','off');

NDemod = length(senalCDemod);                  %Muestras
FourierDemod = fft(senalCDemod, NDemod);         %Espectro mediante fft
LinDemod =linspace(-Fs/2, Fs/2, NDemod);    %Genera n puntos para la grafica
AmpDemod = fftshift( abs( FourierDemod ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
stem(LinDemod, AmpDemod, 'b');              %Grafica la funcion Vam con color azul
axis([0, 1000, 0, 1000]);          %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal Demodulada: Audio Capturado');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');              %Coloca una etiqueta horizontal (eje x)


%---------------------------------------------------------------------%
%                  O)  Agregue ruido a la señal                       %
%---------------------------------------------------------------------%

figure('Name','Agregando Ruido a la señal: Audio Capturado ','NumberTitle','off');

subplot(3, 1, 1);                 %Subdivide la ventana en 4 Filas 1 columna y se posiciona en la Fila 1
k = 2;
VmR = Vm+k;
ruido = k*rand(size(duracion));          %Generar algún ruido aleatorio con una desviación estandar de k para  producir una señal de ruido. 
senalCR = senalC + ruido;                 %Agrega ruido a la señal
plot(senalCR,'r'); %Grafica la funcion Vam con color azul
%axis([5e4, 5e4+300, -k, k]);   %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Señal con ruido');        %Coloca una etiqueta vertical (eje y)


VamR = ammod(senalCR, Fc, Fs);          %Modulacion de la señal
subplot(3,1,2);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 2
plot(VamR,'b','linewidth',grosor/2);%Grafica la funcion Vam con color azul
%axis([5e4, 5e4+300, -k, k]);     %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Modulada');                 %Coloca una etiqueta vertical (eje y)

WnR = Fc*2/Fs;
senalCDemodR = amdemod(VamR,Fc,Fs);         %Demodulacion de la señal amdemod (Vam,Fc,Fs)
subplot(3,1,3);                                              %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 3
%plot(senalC,'r','linewidth',grosor/2);         %Grafica la funcion Vam con color azul
%hold on;
plot(senalCDemodR,'g','linewidth',grosor/2);         %Grafica la funcion Vam con color azul
ylabel('Demodulacion');                              %Coloca una etiqueta vertical (eje y)
%axis([5e4, 5e4+300, -k, k]);                        %Muestra solo la parte de la funcion en los intervalos especificados


figure('Name','Espectros de la señal con ruido: Audio Capturado','NumberTitle','off');

subplot(3, 1, 1);
NR = length(senalCR);                  %Muestras
FourierR = fft(senalCR, NR);         %Espectro mediante fft
LinR =linspace(-Fs/2, Fs/2, NR);    %Genera n puntos para la grafica
AmpR = fftshift( abs( FourierR ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
stem(LinR, AmpR, 'b');              %Grafica la funcion Vam con color azul
%axis([0, 1000, 0, 1000]);          %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal con ruido: Audio Capturado');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');              %Coloca una etiqueta horizontal (eje x)

subplot(3, 1, 2);
NRMod = length(VamR);                  %Muestras
FourierRMod = fft(VamR, NRMod);         %Espectro mediante fft
LinRMod =linspace(-Fs/2, Fs/2, NRMod);    %Genera n puntos para la grafica
AmpRMod = fftshift( abs( FourierRMod ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
stem(LinRMod, AmpRMod, 'b');              %Grafica la funcion Vam con color azul
%axis([0, 1000, 0, 1000]);          %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal Modulada con ruido: Audio Capturado');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');              %Coloca una etiqueta horizontal (eje x)

subplot(3, 1, 3);
NRDemod = length(senalCDemodR);                  %Muestras
FourierRDemod = fft(senalCDemodR, NRDemod);         %Espectro mediante fft
LinRDemod =linspace(-Fs/2, Fs/2, NRDemod);    %Genera n puntos para la grafica
AmpRDemod = fftshift( abs( FourierRDemod ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
stem(LinRDemod, AmpRDemod, 'b');              %Grafica la funcion Vam con color azul
%axis([0, 1000, 0, 1000]);          %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal Demodulada con ruido: Audio Capturado');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');              %Coloca una etiqueta horizontal (eje x)

















