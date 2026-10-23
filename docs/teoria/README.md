# Documentación teórica

Teoría detrás de los modelos del simulador: qué ecuaciones implementa cada
parte del código, de dónde salen y qué supuestos tienen. Un documento LaTeX
independiente por tema; todos comparten `comun/preambulo.tex`.

| Tema | Documento | Código que describe |
|---|---|---|
| Ruido de sensores: pitot, veleta, ISA, generador de ruido | `sensores/ruido_sensores.tex` | `src/sensors_sim.c`, `src/atmosphere.c` |

Próximos: dinámica longitudinal y lateral-direccional del Ugly Stick
(`dinamica_longitudinal/`, `dinamica_lateral/`).

## Compilar

Requiere una distribución TeX con `latexmk` (por ejemplo TeX Live). Desde la
carpeta del documento:

```bash
cd docs/teoria/sensores
latexmk -pdf ruido_sensores.tex      # genera ruido_sensores.pdf
latexmk -c                           # borra auxiliares (deja el PDF)
```

En git solo van las fuentes `.tex`; el PDF y los auxiliares están en
`docs/teoria/.gitignore`.

## Agregar un documento

1. Carpeta nueva `docs/teoria/<tema>/` con `<tema>.tex`.
2. Empezar con:
   ```latex
   \documentclass[11pt,a4paper]{article}
   \input{../comun/preambulo}
   ```
3. Usar las macros del preámbulo para nombres de código (`\archivo{}`,
   `\func{}`, `\clave{}`) y agregarlo a la tabla de arriba.
4. Si el documento describe código, poner en el comentario de cabecera de ese
   código la ruta del `.tex` (como en `src/sensors_sim.c`).
