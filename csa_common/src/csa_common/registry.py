#!/usr/bin/env python3

"""
  A generic Tactic registry object.
  
  Copyright 2026 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


class TacticRegistry(object):
    """
    A registry-pattern object for holding Tactic objects for a module.
    
    Can be used for other purposes such as 'sub'-tactics with a larger
    tactic.
    
    Based on: 'https://dev.to/dentedlogic/stop-writing-giant-if-else-
    chains-master-the-python-registry-pattern-ldm'
    """
    
    def __init__(self, tact_dict=None):
        
        # Intialize storage dict
        self._store = {}
        
        # Register items from optional input dict
        if tact_dict is not None:
            for key, value in tact_dict.items():
                try:
                    self.register(key, value)
                except ValueError as err:
                    print(err)
    
    def __contains__(self, key):
        return key in self._store
    
    def register(self, key, value):
        """
        Add tactic to the registry, with check for existing keys.
        """
        
        if key not in self._store:
            self._store[key] = value
        else:
            raise ValueError("Key {} already registered".format(key))

    def get(self, key):
        """
        Retreive a tactic attached to the given string.
        """
        
        if value:= self._store.get(key):
            return value
        else:
            raise KeyError("Key {} not found in registry".format(key))
            
    def list_keys(self):
        """
        List all tactics stored within the registry.
        """
        
        return(self._store.keys())
