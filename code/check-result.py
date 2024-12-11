from csvdiff import load_csv, compare

# Load the CSV files
diff = compare(
    load_csv(open("results.csv")),
    load_csv(open("instances/min-sum-of-cost.csv"))
)

# Print the differences
print(diff)