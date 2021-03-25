# About the file

The file is a small simulation of the follow paper

https://kns.cnki.net/kcms/detail/11.2127.TP.20200818.1003.002.html

# some details

1.UAV0 is the leader.

2. we did not limit the maximum speed of the vehicle

3. Sometimes they can keep the shape when moving, but most of time they can not

4. the Laplace matrix is
   $$
   L = \left[ \begin{matrix}  
      0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0  
   \\ -1 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0  
   \\ -1 & -1 & 2 & 0 & 0 & 0 & 0 & 0 & 0 & 0  
   \\ 0 & -1 & -1 & 2 & 0 & 0 & 0 & 0 & 0 & 0  
   \\ 0 & 0 & -1 & -1 & 2 & 0 & 0 & 0 & 0 & 0  
   \\ 0 & 0 & 0 & -1 & -1 & 2 & 0 & 0 & 0 & 0  
   \\ 0 & 0 & 0 & 0 & -1 & -1 & 2 & 0 & 0 & 0  
   \\ 0 & 0 & 0 & 0 & 0 & -1 & -1 & 2 & 0 & 0  
   \\ 0 & 0 & 0 & 0 & 0 & 0 & -1 & -1 & 2 & 0  
   \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & -1 & -1 & 2  
   \end{matrix} \right]  
   $$
   

5.The article has some problems(my own opinion) 
