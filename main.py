import numpy as np
import pandas as pd

제품 = list('AB')
대리점 = list('PQ')
공장 = list('XY')
레인 = (2,2)

print(제품, 대리점, 공장, 레인)

# 대리점 - 공장 간 운송비용
trans_cost = pd.DataFrame( ((j,k) for j in 대리점 for k in 공장), columns = ['대리점', '공장'])
trans_cost['운송비'] = [1,2,3,1]
trans_cost