from pyamaze import maze, agent 
m = maze(5, 5)
m.CreateMaze()  
a = agent(m, shape='square', filled=True, footprints=True)

m.tracePath({a: m.path}, delay=100)

m.run()
