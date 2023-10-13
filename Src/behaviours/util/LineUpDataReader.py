
def readData(data_path="/home/nao/data/line_up_data.lud"):
    result = []
    f = open(data_path)
    for line in f:
        line = line.split()
        line_result = []
        for item in line:
            if item == "-":
                line_result.append(0)
            elif item == "/":
                line_result.append(1)
            elif item == "o":
                line_result.append(2)
            else:
                line_result.append(0)
        result.append(line_result[::-1])

    return result[::-1], len(result), len(result[0])
