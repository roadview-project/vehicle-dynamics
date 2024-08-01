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
there is a default vehicle with its parameters in the [YAML](bmw_m8.yaml) file.


# Acknoledgment
Co-funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them. Project grant no. 101069576.
UK participants in this project are co-funded by Innovate UK under contract no.10045139. 
Swiss participants in this project are co-funded by the Swiss State Secretariat for Education, Research and Innovation (SERI) under contract no. 22.00123.


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

[Dataset Creator OSI](https://github.com/roadview-project/dataset_creator_OSI)
- Creates Dataset in ASAM OSI format from rosbags.

