import csv
import random

with open('ncnr_filled.csv', 'rb') as csvfile:
    
    reader = csv.reader(csvfile, delimiter = ',', quotechar ='|')
    
    for row in reader:
        
        if row[19] != '?' and row[0] != 'session':
        
            select = random.randint(1,100)
            
            if select > 80:
                with open('test.csv', 'a') as csvwrite:
                    writer = csv.writer(csvwrite, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([row[13], row[14], row[15], row[16], row[17], row[18], row[19]])
            else:
                with open('train.csv', 'a') as csvwrite:
                    writer = csv.writer(csvwrite, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([row[13], row[14], row[15], row[16], row[17], row[18], row[19]])
        
