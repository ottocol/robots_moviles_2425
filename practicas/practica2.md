# Práctica 2 de Robots Móviles. Mapeado y localización en ROS con filtros de partículas

En esta práctica vamos a probar los algoritmos de mapeado, localización y navegación ya implementados en ROS. Los usados por defecto se basan en **filtros de partículas** como se ve en clase de teoría.

Para crear el mapa, guardarlo y usarlo para localizarse y navegar tendréis que seguir una serie de pasos que no vamos a explicar con detalle aquí, ya que hay multitud de tutoriales en Internet y otros recursos que lo explican y no tiene mucho sentido repetirlo todo aquí. En el [apéndice](#apendice) tenéis unas cuantas referencias, pero podéis usar otras que encontréis en Internet, libros, etc. si lo deseáis.

En el enunciado tenéis requisitos mínimos, que son imprescindibles para aprobar, y requisitos adicionales si queréis obtener más nota.

## Construcción de mapas del entorno (requisito mínimo)

En esta parte tenéis que construir un mapa del entorno, pero ahora con los algoritmos ya implementados en ROS en lugar de usar el vuestro propio. El mapa se usará luego en el siguiente apartado para localizarse y poder navegar por el entorno.

Técnicamente hablando, el algoritmo de mapeado usado por defecto en ROS en realidad es un algoritmo de mapeado y localización simultáneos o SLAM en inglés. Es decir, a la vez que va construyendo el mapa se va localizando para reducir los errores de odometría. 

Mirad en el apéndice las referencias sobre cómo hacer el mapeado según el robot que queráis usar en simulación (Turtlebot 2 o 3). No es necesario usar los dos, basta con uno de ellos.

Pruebas a realizar:

- Construir el mapa en al menos 2 entornos lo más distintos que podáis, por ejemplo uno dentro de un edificio, otro en un espacio abierto con obstáculos más pequeños
- Incluid en la documentación de la práctica los mapas creados
- Si habéis observado problemas en los algoritmos (casos que no funcionen bien, etc) explicadlos también.
- Tened en cuenta que para poder luego localizarse y navegar hay que guardar el mapa en un formato especial. Esto se hace con el comando `rosrun map_server map_saver -f nombre_a_dar_al_mapa` (como seguramente aparecerá en cualquier tutorial de mapeado que uséis).


## Localización y navegación (requisito mínimo)

Una vez tenemos un mapa, lo podemos usar para localizar al robot (saber dónde está en todo momento) y para navegar por el entorno: ir del punto donde está el robot a un destino cualquiera, planificando la mejor trayectoria y sin chocar con obstáculos por el camino. 

De nuevo, las instrucciones concretas para hacer esto dependen del robot y del simulador empleado, tendrás que buscarlas por Internet o consultar las referencias del [apéndice](#apendice).

Pruebas a realizar:

- En los dos entornos en los que hayas construido el mapa prueba a hacer que el robot navegue de forma autónoma a un punto. Ten en cuenta que si la trayectoria es muy larga o pasa por sitios muy estrechos es posible que no funcione bien.
- Documenta la navegación del robot con videos grabados de la pantalla
- Explica en la memoria por escrito qué tal funciona la navegación (si el robot consigue llegar al destino de forma consistente) y la localización (si el robot sabe en todo momento dónde está) y posibles problemas (ya hemos comentado el de los sitios estrechos, si en las pruebas has descubierto otros problemas, coméntalos).

## Parte adicional: probar el mapeado y navegación en los robots reales (hasta 1 punto)

**Los días 23 y 30 de octubre y 6 de noviembre** se podrá probar el mapeado y la localización en los Turtlebot reales. Se harán turnos aleatorios para que se puedan usar los robots en grupos de 2-3 personas, **os puede tocar hacer la prueba uno de estos días (pero no más de uno)**. La lista de turnos se elaborará durante la semana del 14 al 20 de octubre. 

Se publicará una guía indicando los pasos exactos para probar mapeado y navegación en los Turtlebot. En esta guía se indicará qué pruebas debéis hacer y cómo documentarlas. La entrega de estas pruebas se realizará junto con la memoria principal de la práctica. Todos los que hayáis probado juntos el robot tendréis la misma puntuación.

## Parte adicional: análisis de los modelos de movimiento y de observación de ROS (hasta 1 punto)

Como hemos visto en clase de teoría, para poder implementar localización y mapeado basado en filtros de Bayes, que es lo que usa ROS por defecto, necesitamos primero un modelo probabilístico de movimiento y del sensor.

Como el código fuente de ROS está disponible en Internet, en este apartado se trata de que lo examines y compares con lo visto en clase de teoría. Explica lo que hace el código de la forma más detallada que puedas y referencia en todo lo posible las transparencias de teoría relevantes (por ejemplo, "estas líneas calculan la probabilidad de obstáculos inesperados, como se hace en la figura de la derecha de la transparencia X del tema Y...").

- El modelo de observación o del sensor está en [https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl/sensors/amcl_laser.cpp
](https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl/sensors/amcl_laser.cpp
). En concreto el modelo de "haz de luz" que se explicó en teoría está en el método `BeamModel` (en clase de teoría está en las 2 transparencias de teoría "cómo representar matemáticamente los errores", tema 2 parte III) 
- El modelo de movimiento está en [https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl/sensors/amcl_odom.cpp
]( https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl/sensors/amcl_odom.cpp
). En concreto la aplicación del modelo de movimiento muestreado (transparencia de teoría 23, tema 2 parte II) está en el método `UpdateAction`.

Para la máxima nota en este apartado investiga por tu cuenta el modelo basado en scan o *likelihood field model*, explicando cómo funciona en general y cómo funciona en el código fuente.

> Para resolver esta parte os recomendamos que hagáis uso de ChatGPT o algún otro LLM (Modelo del Lenguaje) como Claude o Llama. Copiad y pegad el código relevante y pedid que os lo explique, lo suele hacer bastante bien. Como siempre que uséis un modelo del lenguaje, chequead con lo visto en teoría o con otras fuentes que la explicación tiene sentido y no está inventando cosas. 


## Parte adicional: análisis de un filtro de partículas para localización (hasta 1.5 puntos)

Probad alguna implementación ya hecha de un algoritmo de filtro de partículas para localización. No es necesario que esté integrada en ROS y puede estar implementada en cualquier lenguaje. En Internet hay multitud de ellas. Busca alguna para la que esté disponible el código, y conozcas el lenguaje de programación, ya que deberás examinar su funcionamiento.

Deberías

+ Realizar varias pruebas con el algoritmo variando los parámetros que puedas (como mínimo el número de partículas)
+ Explicar con el máximo detalle que puedas cómo funciona el código identificando las partes que hemos visro en clase (modelo del sensor, modelo de movimiento, bucle del algoritmo, resampling...). Hazlo en forma de comentarios al código fuente.
+ Graba un vídeo de máximo 5 minutos explicando el código y haciendo pruebas del algoritmo. No es necesario que salgas tú directamente :) solo la pantalla 

## Entrega de la práctica

### Baremo

En resumen, el baremo de puntuación es el siguiente:

- **Requisitos mínimos (hasta un 7)**: la práctica debe estar adecuadamente documentada, detallando los resultados de todos los experimentos realizados. 
- **(hasta 1 punto adicional)** probar el mapeado en los Turtlebot reales.
- **(hasta 1 punto adicional)** Análisis de los modelos de movimiento y de observación en el código fuente de ROS 
- **(hasta 1 punto adicional)**: probar alguna implementación ya hecha de un algoritmo de filtro de partículas para localización. 

- En lugar de alguno de los puntos anteriores se puede hacer cualquier otra ampliación que se os ocurra, por ejemplo
	+ (hasta 1 punto) Leer un artículo científico sobre mapeado/localización (estado actual de la investigación, algoritmos alternativos...) haciendo un resumen del contenido. Si no sabéis cuál elegir, preguntad al profesor.
	+ (hasta 1 punto) Probar algún otro algoritmo de mapeado o localización que esté implementado en ROS y esté disponible en Internet. En la memoria de la práctica deberíais incluir cómo lo habéis instalado, una breve explicación de las ideas en que se basa y resultados de funcionamiento, comparándolo con el algoritmo por defecto de ROS e indicando también casos en los que pueda fallar si así ha sido.
	+ **Consultad con el profesor** lo que queréis hacer para ver cuánto se podría valorar en el baremo, si queréis que os pase enlaces a artículos o algoritmos, etc.


### Plazos y procedimiento de entrega

La práctica se podrá entregar hasta las 23:59 horas del **martes 12 de Noviembre**.

La entrega se realizará a través del Moodle de la asignatura. Tiene un tamaño máximo de 20Mb (de momento no lo podemos ampliar), por lo que si necesitáis subir más datos tendréis que poner enlaces a vuestro Google Drive o similar o si son videos, enlaces a Youtube o donde lo podáis subir.

## <a name="apendice">Apéndice: Referencias sobre mapeado y localización en ROS</a>

Para realizar los requisitos mínimos necesitaréis consultar información adicional sobre localización y mapeado en ROS, ya que en clase de prácticas no vamos a explicar las instrucciones exactas para realizarlo. No tenéis por qué seguir exactamente estas referencias que ponemos aquì, podéis usar cualquier otra si os gusta más. No es necesario que hagáis la localización y mapeado simulados con Turtlebot 2 y 3, basta con uno de elos.

> Como recomendación general, fijaos en que la biblioteca de la UA tiene disponibles muchos [libros en formato electrónico](https://cat.biblioteca.ua.es/discovery/search?query=any,contains,ros&pfilter=rtype,exact,books&tab=Coleccion_electronica&search_scope=Coleccion_electronica&vid=34CVA_UA:VU1&offset=0) sobre ROS (cuidado ya que la búsqueda enlazada es de la palabra "ROS" pero aparecerán libros no relacionados). En unos cuantos de ellos se explica el mapeado y la localización en ROS. 

### Turtlebot 2

Si tenéis instalado el soporte de Turtlebot 2 podéis consultar estas referencias. Recordad que los ordenadores del laboratorio tienen este soporte instalado.

Para el simulador **Gazebo** podéis seguir por ejemplo [este tutorial online](https://learn.turtlebot.com/2015/02/03/8/). Recordad que en Turtlebot2/Gazebo el sensor de rango simulado tiene un campo de visión pequeño y los algoritmos no funcionarán tan bien.

### Turtlebot 3

#### Web de Robotis

Si no tenéis los paquetes de Turtlebot 3 instalados podéis ver las [instrucciones de instalación](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) en la web de La empresa Robotis, fabricante de los servos que llevan los Turtlebot 3 (aseguráos de seleccionar vuestra versión de ROS en la barra superior)

Además hay un manual *online* con una sección para [SLAM simulado](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) y otra para [navegación](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/) (de nuevo, aseguráos de seleccionar vuestra versión de ROS).

#### Libro "ROS Robot Programming"

En el libro "ROS robot programming", que está escrito por los desarrolladores del Turtlebot 3 y está [disponible de manera gratuita](https://www.robotis.us/ros-robot-programming-book-digital-copy/) puedes encontrar más información en el capítulo 11. (*Antes se podía descargar directamente con un enlace, ahora parece que hace falta registrarse, es probable que esté disponible en otros sitios web*).


Aunque en el libro lo hace con un turtlebot de verdad, para hacerlo en simulación básicamente se trata de sustituir el comando `roslaunch turtlebot3_bringup turtlebot3_robot.launch`, que arranca el robot real por `roslaunch turtlebot3_gazebo turtlebot3_world.launch`que arranca la simulación.

