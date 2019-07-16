import csv
import numpy
from sklearn.svm import SVC
from sklearn.metrics import confusion_matrix
from sklearn.neural_network import MLPClassifier
import pickle

train = []
trainresult = []
test = []
testresult = []

correct = 0
incorrect = 0

with open('train.csv', 'rb') as csvfile:
    
    trainreader = csv.reader(csvfile, delimiter = ',', quotechar ='|')
    
    layers_nb = 3
    layer_size = 20

    layers = (layer_size, ) * layers_nb
    
    for row in trainreader:
        
        if train == []:
            train = numpy.append(train, [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])])
        else:
            train = numpy.vstack((train, [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])]))
        
        trainresult = numpy.append(trainresult, [row[6]])
        
    #print train
    #print trainresult

    clf = MLPClassifier(hidden_layer_sizes=layers, activation='relu', max_iter=1000, solver="lbgfs")
    clf.fit(train, trainresult) 
    #SVC(C=1.0, cache_size=200, class_weight=None, coef0=0.0, decision_function_shape='ovr', degree=3, gamma='auto', kernel='rbf', max_iter=-1, probability=False, random_state=None, shrinking=True, tol=0.001, verbose=False)
    
    with open('test.csv', 'rb') as csvtestfile:
        
        testreader = csv.reader(csvtestfile, delimiter = ',', quotechar ='|')
        
        for row in testreader:
            
            if test == []:
                test = numpy.append(test, [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])])
            else:
                test = numpy.vstack((test, [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])]))
            
            testresult = numpy.append(testresult, [row[6]])

        print(clf.score(test, testresult))
        predresult = clf.predict(test)
        
        print confusion_matrix(testresult, predresult)
        
    filename = 'mlp_placement.sav'
    pickle.dump(clf, open(filename, 'wb'))
