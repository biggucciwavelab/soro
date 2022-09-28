import matplotlib.pyplot as plt

# %% Check that correct bots are being assigned far left, far right, etc
bot=-1
self.bots[bot].get_angle()
Lc=np.asarray([self.bots[bot].mem[0][0].GetPos().x,self.bots[bot].mem[0][0].GetPos().z])
Lf=np.asarray([self.bots[bot].mem[0][1].GetPos().x,self.bots[bot].mem[0][1].GetPos().z])
Rc=np.asarray([self.bots[bot].mem[1][0].GetPos().x,self.bots[bot].mem[1][0].GetPos().z])
Rf=np.asarray([self.bots[bot].mem[1][1].GetPos().x,self.bots[bot].mem[1][1].GetPos().z])
L = np.asarray([self.bots[bot+1].body.GetPos().x,self.bots[bot+1].body.GetPos().z])
B = np.asarray([self.bots[bot].body.GetPos().x,self.bots[bot].body.GetPos().z])
R = np.asarray([self.bots[bot-1].body.GetPos().x,self.bots[bot-1].body.GetPos().z])

lab=['left close','left far', 'right close', 'right far','bot','left','right']
col=['tab:green','tab:olive','tab:orange','tab:red','tab:gray','tab:blue','tab:red']
scl=[100.,100.,100.,100.,400.,400.,400.]
pos = np.asarray([Lc,Lf,Rc,Rf,B,L,R])

fig,ax=plt.subplots()
for i in range(len(lab)):
	ax.scatter(pos[i,0],pos[i,1],c=col[i],s=scl[i],label=lab[i])

ax.legend()
plt.show()

# %% 
bot=0
self.bots[bot].get_angle()
angle=self.bots[bot].angles
theta=self.bots[bot].body.GetRot().Q_to_Rotv().y