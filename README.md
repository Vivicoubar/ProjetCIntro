# ProjetCIntro

Voici notre projet d'introduction au Département Informatique, réalisé par Oscar Brugne et Victor Matrat.

Nous avons implémenté toutes les fonctions demandées, puis nous avons essayé de pousser la simulation sur plusieurs axes :
  - Gérer la mémoire de nos tableaux et la libérer lors de la fermeture de la fenêtre,
  - Création des BoxCollider et gestion du contact entre une particule et un rectangle2D. Nous avons commencé à travailler sur l'implémentation de la gestion du contact avec un parallélépipède2D, mais la fonction "checkContactWithBox" n'est pas encore adaptée,
  - Nous sommes en train d'implémenter la gestion des contacts entre les particules dans la branche **optimizeContact**. Pour ce faire, nous utilisons un tableau où chaque cellule contient la liste des identifiants des particules qui sont actuellement dans cette case de la simulation. Ce tableau est implémenté dans les fichiers "IntArrayGrid.c" et "IntArrayGrid.h". Une fois le tableau initialisé avec toutes les particules, nous parcourons ce tableau et ne regardons que les contacts potentiels entre une particule dans une cellule et les 9 cellules voisines (y compris elle-même). Actuellement, le problème se trouve dans l'initialisation du tableau. Pour développer cette fonctionnalité et déboguer la fonction actuelle, il faudrait créer un nouveau fichier dédié à cette implémentation pour un code plus lisible.
  - Nous avons créé une planche de Galton dans la branche **FirstGame** pour appliquer notre simulation à un cas concret.

Nous avons pris soin de rendre le code compréhensible en créant la branche **cleanCode** pour améliorer la lisibilité du code. 
La branche **oscar** a permis d'implémenter de nouvelles fonctionnalités par Oscar pendant les séances de cours et nous avons utilisé en parallèle LiveShare lorsque nous souhaitions travailler sur la même fonctionnalité en même temps.

# Récapitulatif des branches 
Les branches **main** et **FirstGame** sont fonctionnelles, avec l'implémentation de la simulation dans **main** et l'application à la planche de Galton dans la branche **FirstGame**.

La branche **optimizeContact** sert à implémenter une nouvelle fonctionnalité.

Les branches **oscar** et **cleanCode** sont des branches qui ont permis de développer des fonctionnalités et de rendre le code plus lisible.
