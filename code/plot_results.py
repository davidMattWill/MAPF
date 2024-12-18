import csv
import matplotlib.pyplot as plt

import statistics


def read_results(filename):
    cost = []
    time = [] 
    num_expand = []
    num_gen = []


    dat = {}
    with open(filename, mode='r') as file:
    # Create a CSV reader object
        csv_reader = csv.reader(file)
    # Iterate over each row in the CSV file
        for line in csv_reader:
            print(line)
            parts = line[0].split()
            num = int(parts[0])


            cost.append(int(parts[1]))
            time.append(float(parts[2]))
            num_expand.append(int(parts[3]))
            num_gen.append(int(parts[4]))
           
     
    return cost, time, num_expand, num_gen, num

cost_1, time_1, num_gen_1, num_expand_1, num_1 = read_results('sipp_results.csv')
cost_2, time_2, num_gen_2, num_expand_2, num_2 = read_results('cbs_results.csv')




print(statistics.mean(time_1))
print(statistics.mean(time_2))

# Example data
x = [i for i in range(1, max(num_1, num_2) + 1)]


# Create scatter plots
plt.scatter(x[0:len(cost_1)], time_1, label='SIPP', color='blue', s=30)
plt.scatter(x[0:len(cost_2)], time_2, label='CBS', color='red', s=30)
# Add titles and labels
plt.title('Number of Agents on a 256x256 Map vs Execution Time')
plt.xlabel('Number of Agents')
plt.ylabel('Execution Time (s)')

# Set x-axis to show only whole numbers
plt.xticks(range(min(x), max(x) + 1))

#plt.ylim(0, 10)
plt.yscale('log')

# Add a legend
plt.legend()

# Show plot
plt.show()


plt.scatter(x[0:len(cost_1)], num_gen_1, label='SIPP', color='blue', s=30)
plt.scatter(x[0:len(cost_2)], num_gen_2, label='CBS', color='red', s=30)
# Add titles and labels
plt.title('Number of Agents on a 256x256 Map vs Node Generations')
plt.xlabel('Number of Agents')
plt.ylabel('Nodes Generated')

# Set x-axis to show only whole numbers
plt.xticks(range(min(x), max(x) + 1))

#plt.ylim(0, 10)
#plt.yscale('log')

# Add a legend
plt.legend()

# Show plot
plt.show()


plt.scatter(x[0:len(cost_1)], num_expand_1, label='SIPP', color='blue', s=30)
plt.scatter(x[0:len(cost_2)], num_expand_2, label='CBS', color='red', s=30)
# Add titles and labels
plt.title('Number of Agents on a 256x256 Map vs Node Expansions')
plt.xlabel('Number of Agents')
plt.ylabel('Node Expansions')

# Set x-axis to show only whole numbers
plt.xticks(range(min(x), max(x) + 1))

#plt.ylim(0, 10)
#plt.yscale('log')

# Add a legend
plt.legend()

# Show plot
plt.show()



