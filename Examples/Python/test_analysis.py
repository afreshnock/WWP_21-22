import decimal
import pandas
import re
import numpy as np
import matplotlib.pyplot as plt
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
plt.style.use('default')

import os
print(os.getcwd())
df_train = pandas.read_excel("Examples/Python/data_4_20_22/6-13.xlsx", sheet_name="raw")
df_test = pandas.read_excel("Examples/Python/data_4_21_22/log176.xlsx", sheet_name="log176")

inputs = ['rpm', 'theta', 'l_power']
equation_map = {
    ' ': '*'
    , 'rpm': '_rpm'
    , 'theta': '_theta'
    , 'resistance': '_resistance'
    , 'l_current': '_l_current'
    , 'l_voltage': '_l_voltage'
    , 'l_power': '_l_power'
    }
output = ['windspeed']
Nt=1
No=1
precision = 'float64'
D=[1, 2, 3, 4, 5, 6, 7, 8]
#D= [2]
for col in df_test.columns:
    df_test[col] = df_test[col].rolling(Nt).mean()
df_test = df_test.iloc[Nt:]


#df_train = df_train.loc[(df_train['windspeed'] > 6) & (df_train['windspeed'] < 9)]
df_train[inputs] = df_train[inputs]#.apply(lambda x: x/1000)
df_train[output] = df_train[output]

#df_test = df_test.loc[(df_test['windspeed'] > 6) & (df_test['windspeed'] < 9)]
df_test[inputs] = df_test[inputs]#.apply(lambda x: x/1000)
df_test['rpm'] = df_test['rpm']#.apply(lambda x: x + 0)
df_test['l_power'] = df_test['l_power'].apply(lambda x: x + 1000)
df_test[output] = df_test[output]

X = df_train[inputs]
y = df_train[output]
X = X.values.astype(precision)
y = y.values.astype(precision)

X_test = df_test[inputs]
y_test = df_test[output]
X_test = X_test.values.astype(precision)
y_test = y_test.values.astype(precision)

res_test = y_test
regr = LinearRegression()

bestScore = 0
topD = 0
for degree in D:
    poly = PolynomialFeatures(degree=degree)
    poly_X = poly.fit_transform(X)
    poly_var_test = poly.fit_transform(X_test)

    model = regr.fit(poly_X, y)
    score = model.score(poly_var_test, res_test)
    print('Degree: ' + str(degree) + ' - ' + 'R^2: ' + str(score))
    if score > bestScore:
        topD = degree
        bestScore = score

print('\nBestDegree: ' + str(topD) + ' - ' + 'R^2: ' + str(bestScore))
poly = PolynomialFeatures(degree=topD)
poly_X = poly.fit_transform(X)
poly_var_test = poly.fit_transform(X_test)

model = regr.fit(poly_X, y)
score = model.score(poly_var_test, res_test)

feature_names = poly.get_feature_names_out(inputs)

names = []
mappattern = re.compile('|'.join(equation_map.keys()))
powpattern = re.compile('^(.*)\^([0-9]*)$')

for item in feature_names:
    replace =  mappattern.sub(lambda x: equation_map[x.group()], item)
    replace = replace.split('*')
    newvar = []
    for p in replace:
        if powpattern.search(p) != None:
            match = powpattern.findall(p)
            powt = 'pow(' + match[0][0] + ', ' + match[0][1] + ')'
            newvar.append(powt)
        else:
            newvar.append(p)
    encl = '(' + '*'.join(newvar) + ')'
    names.append(encl)

equation = str(model.intercept_[0])

i = 0
while i < len(names):
    if precision == 'float32':
        equation += '\n + ' + str(model.coef_[0][i]) + '*' + names[i] 
    else:
        equation += '\n + ' + str(decimal.Decimal.from_float(model.coef_[0][i])) + '*' + names[i] 
    i += 1

#print('inputs: ' + str(feature_names))
#print('coefficients: ' + str(model.coef_))
#print('intercept: ' + str(model.intercept_))

print('windspeed = ' + equation)



df_test['predicted_ws'] = df_test[inputs].apply(lambda s: model.predict(poly.transform(s.values[None]))[0], axis = 1)
df_test['predict_ws_avg'] = df_test['predicted_ws'].rolling(No).mean()
df_test[['windspeed','predicted_ws','predict_ws_avg']].plot(ylim=(4,15))
plt.show(block=False)
input('press <ENTER> to continue')
#df_test.to_excel("Examples/Python/result.xlsx", sheet_name="result") 
