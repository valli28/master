import numpy as np

#This function returns a vector refering to the possible directions the agent can move(nor obstacle nor out of bounds)
def perception(matrix,row,col):
    #first position stands for the possibility of moving up
    #Second for down, third position for left and fourth for right
    array=np.array([False,False,False,False])
    #the maximun positions of the matrix
    superiorBorder=0
    inferiorBorder=((matrix.shape)[0])-1
    leftBorder=0
    rightBorder=((matrix.shape)[1])-1
    
    #The 4 directions we can go
    up=row-1
    down=row+1
    left=col-1
    right=col+1
    #Checks if  moving up will cause an IndexOutOfBound Exception if not ,he will check if there is an obstacle
    #if no obstacle is found it'll make the  movement possible
    if(up>=superiorBorder): #Checks for IndexOutOfBound Exception
        if(matrix[up][col]==0): #Checks for obstacle
            array[0]=True

    if(down<=inferiorBorder): 
        if(matrix[down][col]==0):
            array[1]=True    

    if(left>=leftBorder):
        if(matrix[row][left]==0):
            array[2]=True

    if(right<=rightBorder):
        if(matrix[row][right]==0):
            array[3]=True
    return array

def updateMatrix(matrix,row,col):
    perceptionArray=perception(matrix,row,col)
    #The 4 directions we can go
    up=row-1
    down=row+1
    left=col-1
    right=col+1
    list=[]

    #If possible,the value of the direction we're gonna move will update with the current value of our square +1
    #moving up
    if(perceptionArray[0]):
        matrix[up][col]=matrix[row][col]+1
        list.append(up)
        list.append(col)
    #moving down
    if(perceptionArray[1]):
        matrix[down][col]=matrix[row][col]+1
        list.append(down)
        list.append(col)
    #Moving left
    if(perceptionArray[2]):
        matrix[row][left]=matrix[row][col]+1
        list.append(row)
        list.append(left)
    #Moving right:
    if(perceptionArray[3]):
        matrix[row][right]=matrix[row][col]+1
        list.append(row)
        list.append(right)

    #the list return the coordinates of the squares we can move to
    return list

def completeCoefficientsMatrix(matriz,rowGoal,colGoal):
    nuevosPunto=updateMatrix(matriz,rowGoal,colGoal)
    while(len(nuevosPunto)>0):
        row=nuevosPunto.pop(0)
        col=nuevosPunto.pop(0)
        #se intenta hacer una cola de manera que se de prioridad a los puntos mas prontos a evaluar
        #y la cola se ira reduciendo expulsando el metodo pop, al tener longitud 0 significa que todos los puntos han sido tratados
        nuevosPunto.extend(updateMatrix(matriz,row,col))

#This funcion with the matrix and its coefficients already completed, will only be in charge of getting the shortest Path
def waveFront(matrix,startingRow,StartingCol,EndingRow,EndingCol):
    print("\n -----Coefficients Matrix-------")
    completeCoefficientsMatrix(matrix,EndingRow,EndingCol)
    print(matrix)

    msj="["+str(startingRow)+"]["+str(StartingCol)+"]"
    count=matrix[startingRow][StartingCol]
    # the agent position is at first the starting row and the starting col
    currentRow=startingRow
    currentCol=StartingCol
    #variables for the border of the matrix
    superiorBorder=0
    inferiorBorder=((matrix.shape)[0])-1
    leftBorder=0
    rightBorder=((matrix.shape)[1])-1
    
    while(count!=1):
        #UP
        if(currentRow-1 >= superiorBorder):
            if(matrix[currentRow-1][currentCol]==(matrix[currentRow][currentCol]-1)): #I could've used the count -1 
                currentRow=currentRow-1
                msj=msj+"--->["+str(currentRow)+"]["+str(currentCol)+"]"
                count-=1
                continue
        #DOWN
        if(currentRow+1 <=inferiorBorder):
            if(matrix[currentRow+1][currentCol]==(matrix[currentRow][currentCol]-1)):
                 currentRow=currentRow+1
                 msj=msj+"--->["+str(currentRow)+"]["+str(currentCol)+"]"
                 count-=1
                 continue
        #LEFT
        if(currentCol-1 >=leftBorder):
            if(matrix[currentRow][currentCol-1]==(matrix[currentRow][currentCol]-1)):
                 currentCol=currentCol-1
                 msj=msj+"--->["+str(currentRow)+"]["+str(currentCol)+"]"
                 count-=1

                 continue
        #RIGHT
        if(currentCol+1 <=rightBorder):
            if(matrix[currentRow][currentCol+1]==(matrix[currentRow][currentCol]-1)):
                 currentCol=currentCol+1
                 msj=msj+"--->["+str(currentRow)+"]["+str(currentCol)+"]"
                 count-=1
                 continue
    #print(count)
    print("\n---Shortest Parth from [{},{}] to [{},{}] -----".format(startingRow,StartingCol,EndingRow,EndingCol))
    print(msj)
        


print("------Obstacles Allocation---------")
obstacle=True
default=input("do you want to use default matrix dimension(4x5) and obstacles? y/n: ")
if(default=='y'):
    obstacle=False
    nrows=4
    ncols=5
    matriz=np.zeros((nrows,ncols))
    matriz[1,2]=-1
    matriz[1,3]=-1
    matriz[2,3]=-1
    matriz[3,3]=-1
else:
    nrows=int(input("type the number of row: "))
    ncols=int(input("type the numbers of columns: "))
    matriz=np.zeros((nrows,ncols))
while(obstacle==True):
    obstacleRow=int(input("type the row of the obstacle: "))
    obstacleCol=int(input("type the column of the obstacle: "))
    matriz[obstacleRow][obstacleCol]=-1
    option=input("do you want to add one more obstacle?? y/n: ")
    if(option!="y"):
        obstacle=False

print("\n------Start point------")
rowS=int(input("row of start point----->"))
colS=int(input("col of start point----->"))

print("\n------End point------")
rowE=int(input("row of End point----->"))
colE=int(input("col of End point----->"))
matriz[rowE][colE]=1
waveFront(matriz,rowS,colS,rowE,colE)