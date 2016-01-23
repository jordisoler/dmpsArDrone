# -*- coding: utf-8 -*-
"""
Created on Thu Jan 21 17:56:55 2016

@author: jordi
"""

import Tkinter as tk
root = tk.Tk()

def motion(event):
    x, y = event.x, event.y
    print('{}, {}'.format(x, y))

root.bind('<Motion>', motion)
root.mainloop()