#! /usr/bin/python3

import json
import re
import numpy as np

if __name__ == '__main__':
    data = "[,5 4 3]"
    d_out = re.sub(",\d", r"", data)
    print(d_out)
    json_string = json.dumps(d_out)
    print(json_string)

    array = '{"data": [[[,37.,58.,80.,255.],[,37.,58.,80.,255.],[,36.,57.,79.,255.]]]}'
    arr = np.random.rand(5, 4, 3)
    arr_reshaped = arr.reshape(arr.shape[0], -1)
    print(arr)
    print(arr_reshaped)
    my_data = np.genfromtxt('test.csv', delimiter=',')
    print(my_data)
    print(array)
    data = json.loads(array)

    for element in data['data']:
        print(element)