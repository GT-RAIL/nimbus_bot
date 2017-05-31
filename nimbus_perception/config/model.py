# Load libraries
import pandas as pandas
import matplotlib.pyplot as plt
from pandas.tools.plotting import scatter_matrix
from sklearn import model_selection
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib

# Load dataset
dataset = pandas.read_csv('~/ROS/src/nimbus_bot/nimbus_perception/config/objects.csv')

# Split-out validation dataset
array = dataset.values
X = array[:,0:6]
Y = array[:,6]
validation_size = 0.20
seed = 7
X_train, X_validation, Y_train, Y_validation = model_selection.train_test_split(X, Y, test_size=validation_size, random_state=seed)

# Test options and evaluation metric
seed = 7
scoring = 'accuracy'

# Load model
model = RandomForestClassifier()
model.fit(X_train, Y_train)

# Save the classifier
joblib.dump(model, 'model.pkl')

# Make prediction and print for format check purposes
# print(X_validation)
# print model.predict(X_validation)

#print model.predict([1,1,1,1,1,1])