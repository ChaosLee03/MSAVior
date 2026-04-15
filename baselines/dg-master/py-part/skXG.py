import joblib
import xgboost as xgb
import time
import pandas as pd
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split
from imblearn.over_sampling import SMOTE


df = pd.read_json('modified_data.json', orient='records')

x = df.drop(['Is Related', 'Variable Pair'], axis=1)
y = df['Is Related']
seed = int(time.time())
# 随机1/3设为测试集
# x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.333333, random_state=seed)

# 99.9%都是训练集
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.001, random_state=seed)


# 求出正样本的数量、负样本的数量
positive_samples = sum(y_train == 1)
negative_samples = sum(y_train == 0)

# 设定所需的正负样本比例，是个字典，标签为0的存储这么多，为1的存储这么多
sampling_strategy = {0: negative_samples, 1: positive_samples * 60}

# 创建SMOTE对象，并指定sampling_strategy参数
smote = SMOTE(sampling_strategy=sampling_strategy, random_state=42)

# 进行过采样
x_train_oversampled, y_train_oversampled = smote.fit_resample(x_train, y_train)

positive_samples = sum(y_train == 1)
negative_samples = sum(y_train == 0)

print("Number of positive samples:", positive_samples)
print("Number of negative samples:", negative_samples)


with open('test_labels.txt', 'w') as labels, open('test_features.txt', 'w') as features:
    for row in x_test.values:
        features.write(','.join(str(x) for x in row) + '\n')
    for label in y_test.values:
        labels.write(str(label) + '\n')

model = xgb.XGBClassifier()

model.fit(x_train, y_train)

y_pred = model.predict(x_test)

accuracy = accuracy_score(y_test, y_pred)
print("Accuracy = ", accuracy)

joblib.dump(model, 'xgboost_model.pkl')
