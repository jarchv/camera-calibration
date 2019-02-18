# Calibración de Cámara

### Ejecución:

1. Crear el ejecutable.
```
mkdir build
cd build
cmake ..
make
```

2. Dentro de la carpeta **build**:
```
./out [path/video.avi] 5 4 # 5 y 4 son las dimensiones de la distribuciones de los puntos de control
```

3. Para ejecutar en tiempo real 
```
./out -r 5 4 # 5 y 4 son las dimensiones de la distribuciones de los puntos de control
```
4. Necesariamente el video deberá estar dentro de una carpeta llamada **files**: <br><br>

    .
    ├── ...
    └── repository                   
    │    ├── src
    │    ├── build        
    │    └── files
    │        ├── video1.avi 
    │        ├── video2.avi
    │        .      
    │        └── ...            
    └── ...

<br>

### Resultados

1. Error de reproyeccion:

    | rms      | Cx      | Cy      | fx      | fy      | Test |
    | -------- | ------- | ------- | ------- | ------- | ---- |
    | 0.2851   | 328.038 | 226.529 | 543.055 | 541.609 | 1    |  

2. Reconstruccion de un objecto 3D:

    ![alt text](ringspattern.gif)

    Youtube Vide:

    [![3D reconstruction Rings pattern](https://www.youtube.com/watch?v=pm2XkwcAI60&feature=youtu.be)

3. Observaciones:

    El proceso iterativo mejora considerablemente la calibracion de la camera, pero tomando en cuenta una ponderacion de los puntos
    re-proyectados y los originales, teniendo este ultimo un mayor peso al momento de realizar la ponderación.


### Requerimientos
1. OpenCV 2.4.9 o 3.2.0
2. GCC version 5.4
5. CMake 3.10 [Link](https://www.claudiokuenzler.com/blog/755/install-upgrade-cmake-3.10.1-ubuntu-14.04-trusty-alternatives#.XBCpRhC22kA)