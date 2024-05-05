import pandas as pd
import matplotlib.pyplot as plt

file = 'detections.csv'

# Read the ODS file into a DataFrame
try:
    df = pd.read_csv(file, header=None)
    # print(df)
except Exception as e:
    print("An error occurred:", e)

grouped = df.groupby(0)

# Iterate over each group
# for var1_value, group_df in grouped:
#     # Plot the values from this group
#     plt.figure()
#     plt.plot(group_df.values[1], group_df.columns.values[2], label=f"var1={var1_value}")  # Assuming var2 and var3 are in columns 1 and 2
#     plt.xlabel('var2')
#     plt.ylabel('var3')
#     plt.title(f"Rows with var1={var1_value}")
#     plt.legend()
#     plt.show()

for val, group_df in grouped:
    plt.figure()



