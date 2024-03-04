import os

def calculate_average(file_path):
    total_sum = 0
    lines = 0

    with open(file_path, 'r') as file:
        for line in file:
            total_sum += float(line.strip())
            lines += 1

    return str(total_sum / lines)

for vendor in os.listdir(os.getcwd()):
    if vendor != "log.py":
        print(vendor + ":")

        for scene in os.listdir(vendor):
            print("\t" + scene + ":")

            for algorithm in os.listdir(vendor + "/" + scene):
                print("\t\t" + algorithm + ":")

                for file in os.listdir(vendor + "/" + scene + "/" + algorithm):
                    print("\t\t\t" + file + ": " + calculate_average(vendor + "/" + scene + "/" + algorithm + "/" + file))
                