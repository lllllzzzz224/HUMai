from pathlib import Path
import json, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
root=Path(__file__).parent
r=json.loads((root/'report.json').read_text())
joints={j.find('child').attrib['link']:j for j in ET.parse('/home/li/ros2_ws/install/rm_description/share/rm_description/urdf/rm_65.urdf').getroot().findall('joint')}
chain=[];link='tcp_link'
while link!='base_link':
    j=joints[link];chain.append(j);link=j.find('parent').attrib['link']
def fk(q):
    t=np.eye(4);values={f'joint{i+1}':v for i,v in enumerate(q)}
    for j in reversed(chain):
        origin=j.find('origin');m=np.eye(4)
        m[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        m[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        t=t@m
        if j.get('type') in ('revolute','continuous'):
            m=np.eye(4);m[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*values[j.get('name')]).as_matrix();t=t@m
    return t[:3,3]
trial=r['attempts'][1];qs=np.array(trial['q_start']);qe=np.array(trial['q_goal'])
direct=np.array([fk(qs+(qe-qs)*t) for t in np.linspace(0,1,101)])
samples=json.loads((root/(trial['label']+'.samples.json')).read_text())
path=np.array([fk(s['q']) for s in samples])
box=r['obstacle_trials'][0];center=np.array(box['center_m']);h=box['size_m']/2
verts=np.array([[x,y,z] for x in (-h,h) for y in (-h,h) for z in (-h,h)])+center
faces=[[verts[i] for i in ids] for ids in [(0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5)]]
fig=plt.figure(figsize=(10,7),facecolor='#f7f9fb');ax=fig.add_subplot(111,projection='3d')
ax.add_collection3d(Poly3DCollection(faces,facecolor='#cc5a3e',alpha=.45,edgecolor='#a13e25'))
ax.plot(*direct.T,'--',color='#cc5a3e',lw=2,label='Straight joint interpolation (collision detected)')
ax.plot(*path.T,color='#126a9d',lw=2.5,label='RRTConnect preview (117 sampled states valid)')
ax.scatter(*path[0],color='#2a9258',s=65,label='Start');ax.scatter(*path[-1],color='#24394b',s=65,label='Artificial goal')
ax.set(xlabel='Base X (m)',ylabel='Base Y (m)',zlabel='Base Z (m)')
ax.set_title('Synthetic obstacle test: TCP path preview\nArtificial goal and box; NOT an orange grasp / NOT executable',pad=18,fontsize=13)
ax.legend(loc='upper left',fontsize=8)
bounds=np.vstack([direct,path,verts]);lo=bounds.min(axis=0)-.025;hi=bounds.max(axis=0)+.025
ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(lo[2],hi[2])
ax.set_box_aspect(hi-lo);ax.view_init(elev=27,azim=-60)
fig.text(.08,.025,'Robot collision checks use the installed model. TCP curves alone do not certify the full swept volume.',fontsize=9,color='#415269')
fig.savefig(root/'synthetic_detour.png',dpi=170,bbox_inches='tight');print(root/'synthetic_detour.png')
