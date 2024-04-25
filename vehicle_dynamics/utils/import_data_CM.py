# -*- coding: utf-8 -*-
"""
Created on Fri Jul  7 11:34:11 2023

@author: albertonbloemer
"""


def import_data_CM(path):
    import pickle 
    with open(path, "rb") as handle:
        data = pickle.load(handle)
        return data 


def main():
    path = "../../exampledata/acc_brake/SimulationData.pickle"
    data = import_data_CM(path)
    print(data)

if __name__ == '__main__':
    main()
