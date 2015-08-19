# -*- coding: utf-8 -*-
"""
Created on 21-May-2015
Bobby McKinney

Magnetic Field Interpolator: 
    Inputs are Pole Separation in mm and Current in Amps
    Program interpolates the magnetic field based off a matrix
    of values found from graph for 3" Fe Poles in EM4-HVA manual

"""

def fieldInterp(x,i):
    cur = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
    
    sep = [5, 10, 15, 16.3, 20, 22.9, 25, 28.4, 38.1, 50, 50.8, 75, 100, 110]

    field = [[0., 12.8387, 20.7581, 22.371, 22.9387, 23.4806, 23.7355, 24.0452], 
        [0., 7.09032, 13.2806, 17.6677, 19.7581, 20.9065, 21.4581, 21.8226], 
        [0., 4.89032, 9.28387, 13.2258, 16.0129, 17.6677, 18.5516, 19.3194], 
        [0., 4.5, 8.58065, 12.3935, 15.1032, 16.9516, 17.8484, 18.5806], 
        [0., 3.77742, 7.1129, 10.3323, 13.1, 14.9645, 16.1387, 16.9484], 
        [0., 3.38065, 6.27419, 9.19677, 11.671, 13.6097, 14.9226, 15.9097], 
        [0., 3.07097, 5.76452, 8.3871, 10.8355, 12.8129, 14.2065, 15.0839], 
        [0., 2.71613, 5.14516, 7.51613, 9.59355, 11.6355, 12.9581, 14.0484], 
        [0., 2.05161, 3.90645, 5.65484, 7.39677, 9.01935, 10.3129, 11.4258], 
        [0., 1.70968, 3.05806, 4.41613, 5.66129, 7.08065, 8.17097, 9.17097],
        [0., 1.70958, 3.05796, 4.41623, 5.59032, 6.89355, 8.06452, 9.05161], 
        [0., 1.0871, 2., 2.92258, 3.79677, 4.63871, 5.49032, 6.20968], 
        [0., 0.796774, 1.56129, 2.21935, 2.86774, 3.51613, 4.08387, 4.65161], 
        [0., 0.796674, 1.56119, 2.02581, 2.60968, 3.24839, 3.75484, 4.27419]]
        
    sign = 'pos'
    
    if i < 0:
        sign = 'neg'
        i = abs(i)
    #end if
    
    if i in cur and x in sep:
        B = field[sep.index(x)][cur.index(i)]
        return B
    #end if
    
    elif i in cur and x not in sep:
        ni = cur.index(i)
        if x < sep[-1]:
            for n in range(len(sep)):
                if sep[n] < x < sep[n+1]:
                    nx = n
        #end if
        else:
            nx = -2
        #end else
            
        B = field[nx][ni]+(field[nx+1][ni]-field[nx][ni])*(x-sep[nx])/(sep[nx+1]-sep[nx])           
    #end elif
    
    elif i not in cur and x in sep:
        nx = sep.index(x)
        if i< cur[-1]:
            for n in range(len(cur)):
                if cur[n]< i < cur[n+1]:
                    ni = n
        #end if
        else:
            ni = -2
        #end else
        
        B = field[nx][ni]+(field[nx][ni+1]-field[nx][ni])*(i-cur[ni])/(cur[ni+1]-cur[ni]) 
    #end elif
    
    else:
        if i < cur[-1] and x < sep[-1]:   
            for n in range(len(cur)):
                if cur[n]< i < cur[n+1]:
                    ni = n
        
            for n in range(len(sep)):
                if sep[n] < x < sep[n+1]:
                    nx = n
        #end if
        elif i > cur[-1] and x < sep[-1]:
            ni = -2
            for n in range(len(sep)):
                if sep[n] < x < sep[n+1]:
                    nx = n
        # end elif
        
        elif i < cur[-1] and x > sep[-1]:
            nx = -2
            for n in range(len(cur)):
                if cur[n]< i < cur[n+1]:
                    ni = n
        # end elif
        
        else: 
            nx = -2
            ni = -2
        #end else
        
        B = 1/((sep[nx+1]-sep[nx])*(cur[ni+1]-cur[ni]))* \
        (field[nx][ni]*(sep[nx+1]-x)*(cur[ni+1]-i)+ \
        field[nx+1][ni]*(x-sep[nx])*(cur[ni+1]-i)+ \
        field[nx][ni+1]*(sep[nx+1]-x)*(i-cur[ni])+ \
        field[nx+1][ni+1]*(x-sep[nx])*(i-cur[ni]))
    #end else    
    if sign == 'pos':
        return .1*B
    else:
        return -.1*B
    
                
#end def 

#==============================================================================
            
           
def main():
    x = 40
    i = -75
    
    B = fieldInterp(x,i)
    print B
    
#end def
    
if __name__ == '__main__':
    main()
#end if