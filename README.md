# AI-Project
Course Homework for Artificial Intelligence

estimated time spent on this homework : +50 hours



**Pseudocode of Graph-Search**

    function GRAPH-SEARCH(problem, fringe) return a solution, or failure
        closed <- an empty set
        fringe <- INSERT(MAKE-NODE(INITIAL-STATE[problem]), fringe)
        loop do
            if fringe is empty then return failure
            node <- REMOVE-FRONT(fringe)
            if GOAL-TEST(problem, STATE[node]) then return node
            if STATE[node] is not in closed then
                add STATE[node] to closed
                for child-node in EXPAND(STATE[node], problem) do
                    fringe <- INSERT(child-node, fringe)
                end
        end


#### Question 1. Finding a Fixed Food Dot using Depth First Search

**Strategy: ** 

dfsPath = graphSearch(problem, Stack())

**Result : **

1. tinyMaze
    `python pacman.py -l tinyMaze -p SearchAgent`
    Path found with total cost of 10 in 0.0 seconds
    Search nodes expanded: 15

2. mediumMaze

    `python pacman.py -l mediumMaze -p SearchAgent`
    Path found with total cost of 130 in 0.0 seconds
    Search nodes expanded: 146

3. bigMaze

    `python pacman.py -l bigMaze -z .5 -p SearchAgent`
    Path found with total cost of 210 in 0.0 seconds
    Search nodes expanded: 390



#### Question 2. Breath First Search

**Strategy :**

bfsPath = graphSearch(problem, Queue())

**Result : **

1. mediumMaze
    `python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs`
    Path found with total cost of 68 in 0.0 seconds
    Search nodes expanded: 269

2. bigMaze

    `python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5`
    Path found with total cost of 210 in 0.0 seconds
    Search nodes expanded: 620'



#### Question 3. Varying the Cost Function

**Strategy :**

ucsPath = graphSearch_UCS(problem, PriorityQueue())

​               = graphSearch_aStar(problem, PriorityQueue(), nullHeuristic)

**Result : **

1. mediumMaze:

    `python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs`
    Path found with total cost of 68 in 0.1 seconds
    Search nodes expanded: 269

2. mediumDottedMaze:

    `python pacman.py -l mediumDottedMaze -p StayEastSearchAgent`
    Path found with total cost of 1 in 0.0 seconds
    StayEastSearchAgent : costFn = lambda pos: .5 ** pos[0]
    Search nodes expanded: 186

3. mediumScaryMaze:

    `python pacman.py -l mediumScaryMaze -p StayWestSearchAgent`
    Path found with total cost of 68719479864 in 0.0 seconds
    StayWestSearchAgent : costFn = lambda pos: 2 ** pos[0]
    Search nodes expanded: 108

    ​

#### Question 4. A* search

**Strategy** :

aStarPath = graphSearch_aStar(problem, PriorityQueue(), heuristic)

**Result : **

Use manhattanHeuristic : 

`python pacman.py -l bigMaze -z .5 -p SearchAgent -a`

`fn=astar, heuristic=manhattanHeuristic`

Path found with total cost of 210 in 0.2 seconds

Search nodes expanded: 549



#### Question 5. Finding All the Corners

**Strategy :** 

1. StartState : Use True and False to show whether the corner has been eaten or not. Return the startingPosition and is_corner_eaten list
2. isGoalState : all the element in is_corner_eaten is True
3. If a new position not hit wall and is a corner, update the corner as True in is_corner_eater.

**Result :** 

1. tinyCorners

    `python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,`

    `prob=CornersProblem`
    Path found with total cost of 28 in 0.0 seconds
    Search nodes expanded: 252

2. mediumCorners
    `python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,`
    `prob=CornersProblem`
    Path found with total cost of 106 in 0.0 seconds
    Search nodes expanded: 1966

    ​

#### Question 6. Corner Problem: Heuristic

**Strategy** : 

1. List out all the unvisited corners and compute the Manhattan distance to each of them. 

            distance = manhattanDistance(currentPosition, corner)

2. Then select the corner with minimum Manhattan distance. Note down the distance, this is the minimum number of steps needed to reach the corner irrespective of maze. 

            if distance < manhattanDis:
                manhattanDis = distance
                flag = index
3. Update the current position of pacman to this corner. Remove this corner from the unvisited corners list. 

            update = list(is_corner_eaten)
            update[flag] = True
            remaining_corner = tuple(update)
            state_after_visit = (corners[flag], remaining_corner)`
4. Loop over until the unvisited corners is empty. 

            heuristic_after_visit = cornersHeuristic(state_after_visit, problem)

5. The sum of these distances will be an admissible and consistent Heuristic.

    lower_bound = min(lower_bound, manhattanDis +  heuristic_after_visit)`

**Result** :

`python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5`

​	Path found with total cost of 106 in 0.3 seconds

​	Search nodes expanded: 692



#### Question 7. Eating All the Dots

**Strategy** : foodHeuristic = closeDisBetweenPosFood + farDisBetweenFoods

1. calculate two values

   closeDisBetweenPosFood : the real distance from current Pacman position to the closest food

       for food in foodPositions:
           foodAndPositionDis = mazeDistance(food, currentPosition, gameState)
           if foodAndPositionDis < closeDisBetweenPosFood:
               closeDisBetweenPosFood = foodAndPositionDis
   farDisBetweenFoods : the real distance between two furthest foods in the maze

       for foodA in foodPositions:
           for foodB in foodPositions:
               disBetweenFoods = mazeDistance(foodA, foodB, gameState)
               if disBetweenFoods > farDisBetweenFoods:
                   farDisBetweenFoods = disBetweenFoods
   p.s. the real distance here is not Manhattan distance, but the Maze distance of two point, which is already defined in searchAgents.py

2. Why add the two values?

   In either way, we will have to travel the longest distance between two foods, that is, farDisBetweenFoods, at least at the end.

   It is better to collect the food that is near to us so we don't have to go back.

**Result :**

1. AStarFoodSearchAgent

   a. trickySearch 
   `python pacman.py -l trickySearch -p AStarFoodSearchAgent`
   Path found with total cost of 60 in 70.5 seconds
   Search nodes expanded: 719

   b. mediumSearch
   `python pacman.py -l mediumSearch -p AStarFoodSearchAgent`
   Cannot solve in a short time.

   c. tinySearch
   `python pacman.py -l tinySearch -p AStarFoodSearchAgent`
   Path found with total cost of 27 in 20.5 seconds
   Search nodes expanded: 911

2. UcsFoodSearchAgent

   `python pacman.py -l trickySearch -p UcsFoodSearchAgent`

   Path found with total cost of 60 in 61.8 seconds

   Search nodes expanded: 16688



#### Question 8. Suboptimal Search

**Strategy : **

check if the x, y state is in the food list, if so, that isGoalState. And we can use BFS, UCS, A* in findPathToClosedDot function.

**Result :**

Path found with cost 350.