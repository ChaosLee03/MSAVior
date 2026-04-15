import pandas as pd
import joblib
from sklearn.metrics import accuracy_score
import os
import sys
import json

# 加载模型
model_file = 'xgboost_model.pkl'
if getattr(sys, 'frozen', False):
    model_file = os.path.join(sys._MEIPASS, model_file)
loaded_model = joblib.load(model_file)

# 读取测试数据
data_file = 'modified_data.json'
if getattr(sys, 'frozen', False):
    data_file = os.path.join(sys._MEIPASS, data_file)
df_test = pd.read_json(data_file)

pd.set_option('display.max_colwidth', 1000)

# 加载模型
# loaded_model = joblib.load('xgboost_model.pkl')

# 读取测试数据
# df_test = pd.read_json('modified_data.json')

# 提取特征和标签
X_test = df_test.drop(['Is Related', 'Variable Pair'], axis=1)
y_test = df_test['Is Related']

# 进行预测
y_pred = loaded_model.predict(X_test)

# 输出测试集大小
print(len(y_test))

# 计算准确率
accuracy = accuracy_score(y_test, y_pred)
print("Accuracy:", accuracy)

# 查找被错误预测的样例
incorrect_predictions = df_test[y_test != y_pred]
print("预测错的有")
print(incorrect_predictions, incorrect_predictions['Variable Pair'])

# 查找预测标签为1的样例
positive_predictions = df_test[y_pred == 1]
positive_predictions.loc[:, 'True Label'] = y_test[y_pred == 1].values
print("预测标签为1的样本以及它们的真实标签")
print(positive_predictions[['Variable Pair', 'True Label']])

output_data = positive_predictions[['Variable Pair']].values.tolist()
with open('output_data.json', 'w') as f:
    json.dump(output_data, f)

print("误报率：")
right_predictions = df_test[(y_pred == 1) & (y_test == 1)]
print((len(positive_predictions) - len(right_predictions)) / len(positive_predictions))

print("误报个数：")
print(len(positive_predictions) - len(right_predictions))

print("漏报率")
true_1 = df_test[y_test == 1]
print((len(true_1) - len(right_predictions)) / len(true_1))

print("漏报个数：")
print(len(true_1) - len(right_predictions))