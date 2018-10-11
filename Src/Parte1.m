clear all;
clc;
%---------------------------------------------------------------------%
%                              Datos globales                         %
%---------------------------------------------------------------------%

Fs = 44100;     %Frecuencia de muestreo
t=0:1/Fs:1;     %Vector de tiempo 
n = 5;          %Veces que se repite el periodo en la grafica
grosor = 4;     %Grueso de las lineas para graficar
xMax = 0.02;     %Valor maximo de X para mostrar


%---------------------------------------------------------------------%
%                 B) Grafica de cada tono individual                  %
%---------------------------------------------------------------------%

figure('Name','Tonos Individuales','NumberTitle','off');

Vm1 = 1;                                %Amplitud de la moduladora 1
Fm1 = 394;                              %Frecuencia del tono 1 SOL
T1 = 1/Fm1;                             %Periodo del tono 1
Wm1 = 2*pi*Fm1;                         %Frecuencia angular
Vt1 = Vm1*sin(Wm1*t);                   %Funcion de la señal moduladora del tono 1
subplot(3, 1, 1);                       %Subdivide la ventana en 3 Filas 1 columna y se posiciona en Fila 1
plot(t, Vt1, 'r', 'linewidth', grosor); %Grafica la funcion Vt1 con color rojo
axis([0, xMax, -Vm1, Vm1]);             %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfTono 1: G');                  %Coloca el titulo a la gráfica
xlabel('Tiempo (seg)');                 %Coloca una etiqueta horizontal (eje x)
ylabel('Amplitud (V)');                 %Coloca una etiqueta vertical (eje y)
%sound(Vt1,Fs);                         %Hace sonar el tono

Vm2 = 1;                                %Amplitud de la moduladora 2
Fm2 = 440;                              %Frecuencia del tono 2 LA
T2 = 1/Fm2;                             %Periodo del tono 2
Wm2 = 2*pi*Fm2;                         %Frecuencia angular
Vt2 = Vm2*sin(Wm2*t);                   %Funcion de la señal moduladora del tono 2
subplot(3, 1, 2);                       %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 2
plot(t, Vt2, 'g', 'linewidth', grosor); %Grafica la funcion Vt2 con color verde
axis([0, xMax, -Vm2, Vm2]);             %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfTono 2: A');                  %Coloca el titulo a la gráfica
xlabel('Tiempo (seg)');                 %Coloca una etiqueta horizontal (eje x)
ylabel('Amplitud (V)');                 %Coloca una etiqueta vertical (eje y)
%sound(Vt2,Fs);

Vm3 = 1;                                %Amplitud de la moduladora 3
Fm3 = 494;                              %Frecuencia del tono 3 LA
T3 = 1/Fm3;                             %Periodo del tono 3
Wm3 = 2*pi*Fm3;                         %Frecuencia angular
Vt3 = Vm3*sin(Wm3*t);                   %Funcion de la señal moduladora del tono 3
subplot(3, 1, 3);                       %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 3
plot(t, Vt3, 'b', 'linewidth', grosor); %Grafica la funcion Vt3 con color azul
axis([0, xMax, -Vm3, Vm3]);             %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfTono 3: B');                  %Coloca el titulo a la gráfica
xlabel('Tiempo (seg)');                 %Coloca una etiqueta horizontal (eje x)
ylabel('Amplitud (V)');                 %Coloca una etiqueta vertical (eje y)


%---------------------------------------------------------------------%
%               C)   Grafica de 3 tonos simultaneos                   %
%---------------------------------------------------------------------%

figure('Name','Tonos Simultaneos','NumberTitle','off');

subplot(2, 1, 1);                       %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 1
plot(t, Vt1, 'r', 'linewidth', grosor); %Grafica las funciones Vt2 con color verde
hold on;                                %Mantiene la grafica en pantalla
plot(t, Vt2, 'g', 'linewidth', grosor); %Grafica las funciones Vt2 con color verde
hold on;                                %Mantiene la grafica en pantalla
plot(t, Vt3, 'b', 'linewidth', grosor); %Grafica las funciones Vt3 con color azul
title('\bfTonos simultaneos');          %Coloca el titulo a la gráfica
xlabel('Tiempo (seg)');                 %Coloca una etiqueta horizontal (eje x)
ylabel('Amplitud (V)');                 %Coloca una etiqueta vertical (eje y)
axis([0, xMax, -Vm3, Vm3]);            %Muestra solo la parte de la funcion en los intervalos especificados

Vmt = Vm1+Vm2+Vm3;                      %Amplitud de la señal
Vt = Vt1 + Vt2 + Vt3;                   %Suma los 3 tonos en una sola señal
Fmt = (Fm1+Fm2+Fm3)/3;                  %Pseudofrecuencia de la señal
Tt = 1/Fmt;                             %Pseudoperiodo de la señal
subplot(2, 1, 2);                       %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 2
plot(t, Vt, 'r', 'linewidth', grosor);  %Grafica las funciones Vt2 con color verde
axis([0, xMax, -Vmt, Vmt]);            %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfSeñal: Tonos sumados');       %Coloca el titulo a la gráfica
xlabel('Tiempo (seg)');                 %Coloca una etiqueta horizontal (eje x)
ylabel('Amplitud (V)');                 %Coloca una etiqueta vertical (eje y)
%sound(Vtotal, Fs);

%---------------------------------------------------------------------%
%      D ) y E)    Determine espectro de frecuencia y grafique        %
%---------------------------------------------------------------------%

figure('Name','Espectro de frecuencias','NumberTitle','off');

Mu1 = length(Vt1);                          %Muestras del tono 1 (G)
Fu1 = fft(Vt1, Mu1);                        %Uso de la transformada rapida de fourier para encontrar los componentes de frecuencia de una señal
Fr1 = linspace(-Fs/2, Fs/2, Mu1);           %Genera n puntos para la grafica
Amp1 = fftshift( abs( Fu1 ) );              %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 

Mu2 = length(Vt2);                          %Muestras del tono 2 (A)
Fu2 = fft(Vt2, Mu2);                        %Uso de la transformada rapida de fourier para encontrar los componentes de frecuencia de una señal
Fr2 = linspace(-Fs/2, Fs/2, Mu2);           %Genera n puntos para la grafica
Amp2 = fftshift( abs( Fu2 ) );              %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 

Mu3 = length(Vt3);                          %Muestras del tono 3 (B)
Fu3 = fft(Vt3, Mu3);                        %Uso de la transformada rapida de fourier para encontrar los componentes de frecuencia de una señal
Fr3 = linspace(-Fs/2, Fs/2, Mu3);           %Genera n puntos para la grafica
Amp3 = fftshift( abs( Fu3 ) );              %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 

subplot(2, 1, 1);                           %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 1
stem(Fr1, Amp1, 'r','linewidth',grosor/2);  %Grafica las funciones Vm1 con color rojo
hold on;                                    %Mantiene la grafica en pantalla
stem(Fr2, Amp2, 'g','linewidth',grosor/2);  %Grafica las funciones Vm2 con color verde
hold on;                                    %Mantiene la grafica en pantalla
stem(Fr3, Amp3, 'b','linewidth',grosor/2);  %Grafica las funciones Vm3 con color azul
hold on;                                    %Mantiene la grafica en pantalla
title('\bfEspectro de Frecuencia: G --> Rojo,  A --> Verde, B --> Azul'); %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');                  %Coloca una etiqueta horizontal (eje x)
%ylabel('Amplitud (V)');                    %Coloca una etiqueta vertical (eje y)
axis([Fm1-30, Fm3+30, 10, 400]);            %Muestra solo la parte de la funcion en los intervalos especificados


Mut = length(Vt);                           %Muestras de la señal (Tonos sumados)
Fut = fft(Vt, Mut);                         %Uso de la transformada rapida de fourier para encontrar los componentes de frecuencia de una señal
Frt = linspace(-Fs/2, Fs/2, Mut);           %Genera n puntos para la grafica
Ampt = fftshift( abs( Fut ) );              %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 

subplot(2, 1, 2);                           %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 2
stem(Frt, Ampt, 'b','linewidth',grosor/2);  %Grafica las funciones Vm1 con color rojo
hold on;                                    %Mantiene la grafica en pantalla
title('\bfEspectro de Frecuencia de la señal: Tonos sumados');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');                  %Coloca una etiqueta horizontal (eje x)
%ylabel('Amplitud (V)');                    %Coloca una etiqueta vertical (eje y)
axis([Fm1-30, Fm3+30, 10, 400]);                  %Muestra solo la parte de la funcion en los intervalos especificados


%---------------------------------------------------------------------%
%       F) y G)  Module y Muestre graficamente la señal               %
%---------------------------------------------------------------------%


figure('Name','Modulacion en Am de Señal: Tonos sumados','NumberTitle','off');

Vc = 3;                             %Amplitud de la portadora
MultC = 4;                          %Numero de veces que aumenta la frecuencia de la portadora
Fc = Fmt*MultC;                     %Frecuencia de la portadora debe ser al menos 2 veces mayor a la frecuencia de la señal de informacion
Tc = 1/Fc;                          %Periodo de la portadora
Wc = 2*pi*Fc;                       %Frecuencia angular
Vac = Vc*sin(Wc*t);                 %Funcion de la señal portadora 
Vam_C = (Vc+Vt).* sin(2*pi*Fc.*t);  %Modulacion como en clases
% Vmax = Vc + Vmt;
% Vmin = Vc - Vmt;
% 
% Eusf = (Vmax-Vmin)/4;
% Eisf = Eusf;
% cp = Eusf + Eisf; %Cambio Pico de amplitud
% 
%Vam =(1+Vt).*Vac;

Vam = ammod(Vt, Fc, Fs);            %Modulacion de la señal
subplot(3,1,1);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 1
plot(t, Vt,'r','linewidth',grosor); %Grafica las funciones Vt con color rojo
axis([0, xMax, -Vmt-Vc, Vmt+Vc]);         %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Informacion');              %Coloca una etiqueta vertical (eje y)

subplot(3,1,2);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 2
plot(t, Vac,'g','linewidth',grosor);%Grafica las funciones Vac con color verde
axis([0, xMax, -Vmt-Vc, Vmt+Vc]);           %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Portadora');                %Coloca una etiqueta vertical (eje y)


subplot(3,1,3);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 3
plot(t, Vam_C,'g', t, Vt+Vc, 'r', t, (Vt+Vc)*(-1), 'r','linewidth',grosor/2);       %Grafica la funcion Vam con color azul
axis([0, xMax, -Vmt-Vc, Vmt+Vc]);   %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Modulada');                 %Coloca una etiqueta vertical (eje y)



%---------------------------------------------------------------------%
%   H) e I)  Determine espectro de frecuencia (Modulada) y grafique   %
%---------------------------------------------------------------------%

figure('Name','Espectro de frecuencias de la Señal Modulada: Tonos sumados','NumberTitle','off');

% %--------Ejemplo de modulacion con modelo usado en clase--------------%
% C_Fs = Fs;
% C_t  = t;
% C_Fc =  Fc;
% C_Fm = Fmt;
% C_Ec = Vc;
% C_Em = Vmt;
% C_Emoduladora = C_Em * sin(2*pi*C_Fm*C_t);
% C_N = length(C_Emoduladora);                % Muestras
% C_A = C_Ec + C_Emoduladora;                 % Creacion de la envolvente
% C_m = C_A.*sin(2*pi*C_Fc*C_t);              % Modulacion
% C_MF = 2/C_N*abs(fft(C_m, C_N));            % Espectro mediante fft
% C_f = C_Fs * (0 : C_N/2) / C_N;             % Analisis del espectro
% subplot(2, 1, 1);                           %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 1
% stem(C_f(1:(Fs/2)+1), C_MF (1:(Fs/2)+1), 'b');    %Grafica la funcion Vam con color azul
% axis([C_Fc-C_Fm-200, C_Fc+C_Fm+200, 0.7, 3]); %Muestra solo la parte de la funcion en los intervalos especificados
% title('\bfEspectro de Frecuencia de la señal modulada: Tonos sumados');       %Coloca el titulo a la gráfica
% xlabel('Frecuencia (Hz)');                  %Coloca una etiqueta horizontal (eje x)

%--------Ejemplo de modulacion con modelo usado en clase--------------%

C2_Vam = (Vc+Vt).* sin(2*pi*Fc.*t);     %Modulacion
C2_N = length(C2_Vam);                  %Muestras
C2_Fourier = fft(C2_Vam, C2_N);         %Espectro mediante fft
C2_Lin =linspace(-Fs/2, Fs/2, C2_N);    %Genera n puntos para la grafica
C2_Amp = fftshift( abs( C2_Fourier ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
%subplot(2, 1, 2);                       %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 2
stem(C2_Lin, C2_Amp, 'b');              %Grafica la funcion Vam con color azul
axis([Fc-Fmt-100, Fc+Fmt+100, 500, 50000]);          %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal modulada: Tonos sumados');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');              %Coloca una etiqueta horizontal (eje x)


%---------------------------------------------------------------------%
%         J), K) e L)  Demodule, muestre y compare la señal           %
%---------------------------------------------------------------------%

figure('Name','Demodulacion de la señal: Tonos sumados','NumberTitle','off');

Wn = Fc*2/Fs;
[numerador,denominador] = butter(10,Wn);                   %Devuelve los coeficientes de la función de transferencia de un filtro de paso bajo Butterworth digitales de orden 10 con frecuencia de corte normalizado Wn.
VtDemod = amdemod(Vam,Fc,Fs,0,0 ,numerador,denominador);   %Demodulacion de la señal amdemod(Vam,Fc,Fs,fase_inicial,0,numerador,denominador)
plot(t,Vt,'c',t,VtDemod,'r--','linewidth',grosor*2);       %Grafica la funcion Vam con color azul
legend('Señal Original (Tonos Sumados)','Señal (Tonos Sumados) Demodulada'); %Coloca una leyenda a la grafica
xlabel('Tiempo (seg)');                                    %Coloca una etiqueta horizontal (eje x)
ylabel('Amplitud (V)');                                    %Coloca una etiqueta vertical (eje y)
title('\bfDemodulacion de la señal: Tonos sumados');       %Coloca el titulo a la gráfica
axis([0, n*Tt*2, -Vmt, Vmt]);                              %Muestra solo la parte de la funcion en los intervalos especificados

%---------------------------------------------------------------------%
%           M)  Agregue ruido y realice la modulacion                 %
%---------------------------------------------------------------------%

figure('Name','Agregando Ruido a la señal: Tonos sumados','NumberTitle','off');

subplot(3, 1, 1);                 %Subdivide la ventana en 4 Filas 1 columna y se posiciona en la Fila 1
k = 2;
VmtR = Vmt+k;
ruido = k*rand(size(t));          %Generar algún ruido aleatorio con una desviación estandar de k para  producir una señal de ruido. 
VtR = Vt + ruido;                 %Agrega ruido a la señal
VamR = ammod(VtR, Fc, Fs);          %Modulacion de la señal
plot(t,VtR,'red', 'linewidth',2); %Grafica la funcion Vam con color azul
axis([0, xMax, -VmtR-k-Vc, VmtR+k+Vc]);   %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Señal con ruido');        %Coloca una etiqueta vertical (eje y)
xlabel('Tiempo (seg)');           %Coloca una etiqueta horizontal (eje x)


Vam_CR = (Vc+VtR).* sin(2*pi*Fc.*t);  %Modulacion como en clases
subplot(3,1,2);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 2
plot(t, Vam_CR,'b','linewidth',grosor/2);%Grafica la funcion Vam con color azul
axis([0, xMax, -VmtR-k-Vc, VmtR+k+Vc]);     %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Modulada');                 %Coloca una etiqueta vertical (eje y)

subplot(3,1,3);                     %Subdivide la ventana en 3 Filas 1 columna y se posiciona en la Fila 3
plot(t, Vam_CR,'b', t, VtR+Vc, 'r', t, (VtR+Vc)*(-1), 'r','linewidth',grosor/2);       %Grafica la funcion Vam con color azul
axis([0, xMax, -Vmt-Vc-k, VmtR+Vc+k]);   %Muestra solo la parte de la funcion en los intervalos especificados
ylabel('Modulada');  


figure('Name','Demodulacion de la señal con ruido: Tonos sumados (Filtro Butterworth)','NumberTitle','off');

WnR = Fc*2/Fs;
[numeradorR,denominadorR] = butter(10,WnR);                  %Devuelve los coeficientes de la función de transferencia de un filtro de paso bajo Butterworth digitales de orden 10 con frecuencia de corte normalizado Wn.
VtDemodR = amdemod(VamR,Fc,Fs,0,0 ,numeradorR,denominadorR); %Demodulacion de la señal amdemod (Vam,Fc,Fs,fase_inicial,0,numerador,denominador)
plot(t,Vt,'c',t,VtDemodR,'r--','linewidth',grosor);         %Grafica la funcion Vam con color azul
legend('Señal Original (Tonos Sumados)','Señal Demodulada (Butterworth)');    %Coloca una leyenda a la grafica
xlabel('Tiempo (seg)');                                      %Coloca una etiqueta horizontal (eje x)
ylabel('Demodulacion');                                      %Coloca una etiqueta vertical (eje y)
axis([0, n*Tt*2, -VmtR, VmtR]);                            %Muestra solo la parte de la funcion en los intervalos especificados


figure('Name','Espectros de la señal con ruido: Tonos sumados','NumberTitle','off');

CR_N = length(VtR);                     %Muestras
CR_Fourier = fft(VtR, CR_N);         %Espectro mediante fft
CR_Lin =linspace(-Fs/2, Fs/2, CR_N);    %Genera n puntos para la grafica
CR_Amp = fftshift( abs( CR_Fourier ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
subplot(3, 1, 1);                           %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 2
stem(CR_Lin, CR_Amp, 'r','linewidth',grosor/2);  %Grafica las funciones Vm1 con color rojo
hold on;                                    %Mantiene la grafica en pantalla
title('\bfEspectro de Frecuencia de la señal con ruido: Tonos sumados');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');                  %Coloca una etiqueta horizontal (eje x)
%ylabel('Amplitud (V)');                    %Coloca una etiqueta vertical (eje y)
%axis([-1000, 1000, 10, 50000]);             %Muestra solo la parte de la %funcion en los intervalos especificados
axis([Fm1-30, Fm3+30, 10, 400]);


%--------Ejemplo de modulacion con modelo usado en clase--------------%

CR_Vam = (Vc+VtR).* sin(2*pi*Fc.*t);              %Modulacion
CR_NVam = length(CR_Vam);                         %Muestras
CR_FourierVam = fft(CR_Vam, CR_NVam);             %Espectro mediante fft
CR_LinVam =linspace(-Fs/2, Fs/2, CR_NVam);        %Genera n puntos para la grafica
CR_AmpVam = fftshift( abs( CR_FourierVam ) );     %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
subplot(3, 1, 2);                                 %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 1
stem(CR_LinVam, CR_AmpVam, 'g');                  %Grafica la funcion Vam con color verde
axis([Fc-Fmt-100, Fc+Fmt+100, 500, 50000]);       %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal con ruido modulada');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');                        %Coloca una etiqueta horizontal (eje x)

CR_VamDemod = (Vc+VtDemodR).* sin(2*pi*Fc.*t);    %Modulacion
CR_NDemod = length(CR_VamDemod);                  %Muestras
CR_FourierDemod = fft(CR_VamDemod, CR_NDemod);    %Espectro mediante fft
CR_LinDemod =linspace(-Fs/2, Fs/2, CR_NDemod);    %Genera n puntos para la grafica
CR_AmpDemod = fftshift( abs( CR_FourierDemod ) ); %Reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
subplot(3, 1, 3);                                  %Subdivide la ventana en 2 Filas 1 columna y se posiciona en la Fila 1
stem(CR_LinDemod, CR_AmpDemod, 'b');              %Grafica la funcion Vam con color azul
axis([Fc-Fmt-100, Fc+Fmt+100, 500, 50000]);       %Muestra solo la parte de la funcion en los intervalos especificados
title('\bfEspectro de Frecuencia de la señal con ruido demodulada');       %Coloca el titulo a la gráfica
xlabel('Frecuencia (Hz)');                        %Coloca una etiqueta horizontal (eje x)












































































% subplot(3,1,3);
% wavwrite(Vt,Fs,'tonoTotal.wav');
% 
% [VtotalL, Fm1] = wavread('tonoTotal');
% Mu1 = length(VtotalL);                                 %Muestras
% Fu1 = fft(VtotalL, Mu1);                                %uso de la transformada rapida de fourier para encontrar los componentes de frecuencia de una señal
% Fr1 = linspace(-Fm1/2, Fm1/2, Mu1);                     %genera n puntos para la grafica
% Po1 =fftshift( abs( Fu1 ) );                            %reorganiza las salidas de fft moviendo el componente de frecuencua cero hasta el centro de la matriz , util para la vizualuzacion de la transformada de fourirer 
% 
% stem(Fr1, Po1, 'red');
% hold on;
% 
% title('Espectro de Frecuencia: Vtotal');
% xlabel('recuencia (hz)');
% axis([360, 530, 10, 400]); 

