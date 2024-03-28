# Created on : Mar 27, 2024, 10:28:15 PM
# Author     : michael

class PASList:
    def __init__(self):
        self.forward = {}
        self.backward = {}
    
    def containsKey(self, ij):
        return ij in self.forward or ij in self.backward
    
    def add(self, pas):
        ij = pas.getEndLinkBwd()
        if ij not in self.backward:
            self.backward[ij] = set()
        
        self.backward[ij].add(pas)
        
        ij = pas.getEndLinkFwd()
        if ij not in self.forward:
            self.forward[ij] = set()
        
        self.forward[ij].add(pas)
    
    