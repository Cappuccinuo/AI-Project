# AI-Project
Course Homework for Artificial Intelligence

estimated time spent on this homework : +20 hours




#### Question 1. Reflex Agent

**Strategy: ** 

Calculate the minimam manhattan distance to the successor ghost and food.  The food score is 10. 

value = current score + foodScore / minDistanceToFood - foodScore / minDistanceToGhost

As the hint in the material, we can use the reciprocal instead of the value itself. So my method means, the closer we get from food and the further we get from ghost, the higher the score will be. I assigned the same weight to minDistanceToFood and minDistanceToGhost, so the pacman will not go for a food over a ghost, or just wandering around without eating a food.



#### Question 5. Evaluation Function

**Strategy :** 

Combine a linear combination of:

1. distance to the closest active ghost

   -20.0 / minDisToTerroGhost : the further between pacman and active ghost, the larger the score is, otherwise the smaller the score will be.

2. distance to the closest scared ghost

   -200 * minDisToScaredGhost: cause the pacman will earn 200 when eat a scared ghost, and the closer between pacman and the scared ghost, the value will be larger. Since the minDisToScaredGhost can be 0, it cannot use reciprocal here.

3. distance to the closest food

   10.0 / minDisToFood: as the pacman will earn 10 when eat a food, the closer the pacman to the food, the score will be higher.

4. number of foods left

   -25 * numOfFoodLeft: pacman needs to eat as much food as possible, so the less the food left, the higher the score will be.

5. number of capsules left

   -100 * numOfCapsulesLeft: Capsules have higher scores, so I assign a high weight to the capsule. But not as much as the scared ghost. So when the pacman pass by the food, they are motivated to eat the close capsules. 

   ​


#### Question 2. Minimax

**Pseudocode of Minimax**

```
def minimax(game_state):
  moves = game_state.get_available_moves()
  best_move = moves[0]
  best_score = float("-inf")
  for move in moves:
    clone = game_state.next_state(move)
    score = min_play(clone)
    if score > best_score:
      best_move = move
      best_score = score
  return best_move

def min_play(game_state):
  if game_state.is_gameover():
    return evaluate(game_state)
  moves = game_state.get_available_moves()
  best_score = float("inf")
  for move in moves:
    clone = game_state.next_state(move)
    score = max_play(clone)
    if score < best_score:
      best_move = move
      best_score = score
  return best_score
  
def max_play(game_state):
  if game_state.is_gameover():
    return evaluate(game_state)
  moves = game_state.get_available_moves()
  best_score = float("-inf")
  for move in moves:
    clone = game_state.next_state(move)
    score = min_play(clone)
    if score > best_score:
      best_move = move
      best_score = score
  return best_score
```



#### Question 3. Alpha-Beta pruning

Alpha-Beta pruning Pseudocode

```
alpaBetaMinimax(node, alpha, beta) 

   """ 
   Returns best score for the player associated with the given node.
   Also sets the variable bestMove to the move associated with the
   best score at the root node.  
   """

   # check if at search bound
   if node is at depthLimit
      return staticEval(node)

   # check if leaf
   children = successors(node)
   if len(children) == 0
      if node is root
         bestMove = [] 
      return staticEval(node)

   # initialize bestMove
   if node is root
      bestMove = operator of first child
      # check if there is only one option
      if len(children) == 1
         return None

   if it is MAX's turn to move
      for child in children
         result = alphaBetaMinimax(child, alpha, beta)
         if result > alpha
            alpha = result
            if node is root
               bestMove = operator of child
         if alpha >= beta
            return alpha
      return alpha

   if it is MIN's turn to move
      for child in children
         result = alphaBetaMinimax(child, alpha, beta)
         if result < beta
            beta = result
            if node is root
               bestMove = operator of child
         if beta <= alpha
            return beta
      return beta
```

​		

#### Question 4. Expectimax

**Strategy** :

Similar to minimax. Just when the agent is not pacman, use average number to represent its score. That means, max nodes as in minimax search, chance nodes take average(expectation) of value of children.
