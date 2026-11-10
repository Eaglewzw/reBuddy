#!/usr/bin/env python3
"""Plot exported joint commands only; no robot SDK or hardware imports."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder=Path(sys.argv[1])
report=json.loads((folder/'report.json').read_text())
data=np.loadtxt(folder/'trajectory.csv',delimiter=',',skiprows=1)
fig,axes=plt.subplots(3,1,figsize=(16,10),sharex=True)
for row,(ax,label) in enumerate(zip(axes,('Position (deg)','Velocity (deg/s)','Acceleration (deg/s^2)'))):
    for j in range(7):ax.plot(data[:,0],np.degrees(data[:,1+row*7+j]),label=f'J{j+1}',lw=1)
    for piece in report['layout']:
        if piece['kind']=='bridge':
            a=piece['start_s']/report['speed'];b=a+piece['duration_s']/report['speed']
            ax.axvspan(a,b,color='grey',alpha=.16)
        elif row==0:
            ax.text((piece['start_s']+.5*piece['duration_s'])/report['speed'],1.02,piece['number'],
                    transform=ax.get_xaxis_transform(),ha='center',fontsize=9)
    ax.set_ylabel(label);ax.grid(alpha=.2)
axes[0].legend(ncol=7,loc='upper left');axes[-1].set_xlabel('Time (s); shaded intervals = quintic bridges')
fig.suptitle('Dance sequence: '+' - '.join(report['numbers'])+f' | speed {report["speed"]:.0%}',y=.995)
fig.tight_layout();fig.savefig(folder/'trajectory.png',dpi=130);plt.close(fig)
