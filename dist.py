#python 画概率密度图
#-*- coding: utf-8 -*-
# from bdb import _TraceDispatch

from matplotlib import scale
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import statistics as st
# 1）准备数据
# lengths = []
# with open("D:/length_analysis.tsv","r") as f:
#     for l in f:
#         if int(l.split('\t')[2])>80:
#             continue
#         lengths.append(int(l.split('\t')[2])//2)
dir= "/home/zhouyum/catkin_ws/src/ESVO/res_n/res_fly3.txt"
pp=[]
# pp=np.array()
it =0 
with open(dir,'r') as file :
    line =file.readline()
    x = line.split(' ')
    for i in x :
        if i==' ':
            print(i)
            continue
        else :
            # if float(i)==0 and it%2==0:
            #     continue
            try:
                if float(i)<-1000 or float(i)>1000:
                    continue  
                pp.append(float(i))
            except Exception as e :
                print(e)
            it+=1
        # if it>1e4+8011 :
        #     break
print(" variance of list ={} ".format(st.variance(pp)))
print(" mean value of list = {}".format(st.mean(pp)))
print(" standard deviation of list = {}".format(st.stdev(pp)))
# np.var(pp)
# np.mean(pp)
mean =np.mean(pp)
mine= np.std(pp)
Tdist =np.random.standard_t(df=len(pp)-1,size=len(pp))
normale = np.random.normal(loc=0,scale=np.std(pp),size=len(pp))
std_normal=np.random.standard_cauchy(size= len(pp))
poisson = np.random.poisson(lam=len(pp),size=len(pp))
c2= np.random.chisquare(df=1,size=len(pp))

print("size of pp ={}  else get {} ".format(len(pp),it))
# print()
# 2）设置内置背景style
#plt.style.use('seaborn')
# 添加网格显示
plt.grid(linestyle="--", alpha=0.5,linewidth=1.5) 

# 3）画图
bins=[0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]
sns.kdeplot(pp,x='redisual',color='royalblue',fill=True)

#royaleblue 
sns.kdeplot(Tdist,color='black')
sns.kdeplot(normale,color='green')
# sns.kdeplot(c2,color='brown')
# sns.kdeplot(poisson,color='yellow')
# 4）调整
# 修改x轴刻度显示
plt.xticks(range(0, 5)[::5] ,fontsize=10)
# 修改刻度值大小
plt.tick_params(labelsize=6)
# # 添加x, y轴描述信息
plt.xlabel("residual")
plt.ylabel("probability")

plt.show()
print("current dataset dir ={}".format(dir))