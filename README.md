# Calibración de Cámara

### Crear el ejecutable

```
mkdir build
cd build
cmake ..
make
```

### Ejecución:

1. Dentro de la carpeta **build**:
```
./out [path/video.avi] 5 4 # 5 y 4 son las dimensiones de la distribuciones de los puntos de control
```
Necesariamente el video deberá estar dentro de una carpeta llamada **files**: <br><br>

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

### Requerimientos
1. OpenCV 2.4.9 o 3.2.0
2. GCC version 5.4
5. CMake 3.10 [Link](https://www.claudiokuenzler.com/blog/755/install-upgrade-cmake-3.10.1-ubuntu-14.04-trusty-alternatives#.XBCpRhC22kA)