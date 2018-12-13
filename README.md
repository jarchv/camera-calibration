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
./out [video.avi]
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
