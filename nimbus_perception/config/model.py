# Load libraries
import sys
import argparse
import pandas as pandas
import matplotlib.pyplot as plt
from pandas.tools.plotting import scatter_matrix
from sklearn import model_selection
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib

# parameters
dataset = ""
model_name = ""

# get arguments
if len(sys.argv) == 1:
	raise ValueError("dataset not provided")
elif not sys.argv[1].endswith(".csv"):
	raise argparse.ArgumentTypeError("dataset filename must be of type *.csv")
else:
	dataset = sys.argv[1]
	if len(sys.argv) >= 3:
		model_name = sys.argv[2]

# Load dataset
if dataset == "":
	raise ValueError("dataset not valid")
else:
	dataset = pandas.read_csv(dataset)

# Split-out validation dataset
array = dataset.values
X = array[:,0:6]
Y = array[:,6]
validation_size = 0.20
seed = 7
X_train, X_validation, Y_train, Y_validation = model_selection.train_test_split(X, Y, test_size=validation_size, random_state=seed)

# Test options and evaluation metric
seed = 7
scoring = "accuracy"

# Load model
model = RandomForestClassifier()
model.fit(X_train, Y_train)

# Save the classifier
if model_name == "":
	joblib.dump(model, "model.pkl")
else:
	joblib.dump(model, model_name + ".pkl")

# Make prediction and print for format check purposes
# print(X_validation)
# print model.predict(X_validation)

#print model.predict([1,1,1,1,1,1])