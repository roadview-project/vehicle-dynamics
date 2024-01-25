# vehicle-dynamics

Developed by Maikol Funk Drechsler & Yuri Poledna(Technische Hochschule Ingolstadt - CARISSMA Institute of Automated Driving) with assistance of Mattias Hjort and Sogol Kharrazi (VTI - Statens Vag- och Transportforskningsinstitut)

This work also has utils that allow for optimization of your vehicle.

Under the [ROADVIEW Consortium](https://roadview-project.eu/).

# Instalation
``` 
python setup.py sdist bdist_wheel
pip install --editable .
``` 

# Usage
For first use you can simply run
``` 
python main.py
``` 
there is a default vehicle with its parameters in the [YAML](Audi_R8.yaml) file.

For validation some simple scenarios were run and the data is saved under the [exampledata](exampledata) folder.

# Acknoledgment
Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.

# Citation (TODO)
```
@Journal{
	author = {Poledna,Yuri and Drechsler, Maikol Funk and Kharrazi, Sogol and Hjort, Mattias and Huber, Werner},
	booktitle = {},
	year = {},
	volume = {},
	number = {},
	pages = {},
	doi = {},
}
```
# Other ROADVIEW-Project Works:

[REHEARSE Dataset](https://s3.ice.ri.se/roadview-WP3-Warwick/T3.2%20-%20Create%20Dataset/rehearse/index.html)
 - The REHEARSE dataset is a dataset made for sensor models creation and validation, made under extreme weather in controllable conditions.
