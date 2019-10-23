
class InstructionGenerator(object):

    def __init__(self, options = None):
        #set the options
        default_options = {
            'distance_flag':True,
            'endpoint_flag':True,
            'passpoint_flag':True,
        }
        self._options = default_options if options is None else options


    @staticmethod 
    def _generate_single_turn_text(chunk):
        """ Generate text that describe the turn the user going to make.
        """
        if(chunk['clock'] == 3):
            turn_text = 'turn right'
        elif(chunk['clock'] == 9):
            turn_text = 'turn left'
        elif(chunk['clock'] == 0 or chunk['clock'] == 12):
            turn_text = 'keep going straight'
        else:
            turn_text = 'turn to {} o clock'.format(int(chunk['clock']))  

        return turn_text


    @staticmethod 
    def _generate_single_minimum_turn_text(chunk):
        """ Generate minimum text
        """
        if(chunk['clock'] == 3):
            turn_text = 'right'
        elif(chunk['clock'] == 9):
            turn_text = 'left'
        elif(chunk['clock'] == 0 or chunk['clock'] == 12):
            turn_text = 'straight'
        else:
            turn_text = '{} o clock'.format(int(chunk['clock']))  

        return turn_text        

        return text


    @staticmethod
    def _generate_single_translation_text(chunk, desc_type = 'time'):
        """Generate human understandable text on how far to move 
        """
        if desc_type == 'time':
            #220 feet per minute if we used a 2.5 miles per hour walking speed
            #4.6 feet per second
            time_count = int(chunk['dist']/4.6)
            dist_text = ' {} seconds'.format(int(time_count))
        elif desc_type == 'steps':
            #0.4356 steps per feet if we used the average stride length
            step_count = int(chunk['dist'] * 0.4356)
            dist_text = ' {} {}'.format(step_count, 'steps' if step_count > 1 else 'step')
        else:
            dist_text = ' {} feet'.format(int(chunk['dist']))

        walk_text = 'walk for {}'.format(dist_text)            
        return walk_text

    @staticmethod
    def generate_basic_text(chunk):
        """
        """
        turn_text = InstructionGenerator._generate_single_minimum_turn_text(chunk)
        walk_text = InstructionGenerator._generate_single_translation_text(chunk, desc_type='dist')

        return walk_text, turn_text

    @staticmethod
    def generate_text(chunk):
        """
        """
        turn_text = InstructionGenerator._generate_single_turn_text(chunk)
        walk_text = InstructionGenerator._generate_single_translation_text(chunk, desc_type='dist')
        cue_text = chunk.get('cue',"")

        return walk_text, turn_text, cue_text


    # def _generate_junction_endpoint_text(self, chunk, side, _type):
    #     """Generate the text for hallways endpoint where it's a junction
    #     """
    #     # count the amount of hallways you passed
    #     passed_count = 1
    #     if "passed_hallways" in chunk and len(chunk['passed_hallways']) > 0:
    #         for i, passed_hallway in enumerate(chunk['passed_hallways']):
    #             if passed_hallway == _type:
    #                 passed_count += 1
    #             elif passed_hallway == 7: #this means openning to both side
    #                 passed_count += 1
    #     endpoint_text = 'Until you reach the {} openning to your {}'.format(
    #         inflect_tools.ordinal(passed_count), side)        
    #     return endpoint_text

    # def _generate_endpoint_text(self, chunk):
    #     """Generate the human understandable text on at what kind of hallway
    #     should the user turn/end
    #     """
    #     # if the next node is the end of the hallway, either there are just
    #     # one direction to go to or two ways but you will hit the hall
    #     if chunk['next_type'] == 0:
    #         endpoint_text = 'Until you reach the end of the corridor.'
    #     elif chunk['next_type'] == 3:
    #         endpoint_text = ""
    #     elif chunk['next_type'] == 4:
    #         endpoint_text = 'Until you reach the end of the hallway.'
    #     elif chunk['next_type'] == 2:
    #         endpoint_text = 'Until you reach the end of the hallway.'
    #     elif chunk['next_type'] >= 5:
    #         #this is tricky, because this means it is one of the openning to your left
    #         if chunk['next_type'] == 5:
    #             endpoint_text = self._generate_junction_endpoint_text(
    #                 chunk, 'left', 5)
    #         elif chunk['next_type'] == 6:
    #             endpoint_text = self._generate_junction_endpoint_text(
    #                 chunk, 'right', 6)
    #         elif chunk['next_type'] == 7:

    #             if 'next_clock' in chunk:
    #                 next_clock = chunk['next_clock']
    #                 if next_clock == 9:
    #                     endpoint_text = self._generate_junction_endpoint_text(
    #                         chunk, 'left', 5)
    #                 else:
    #                     endpoint_text = self._generate_junction_endpoint_text(
    #                         chunk, 'right', 6)       
    #             else:
    #                 endpoint_text = 'Until you reach openning on both sides'


                                    
    #     elif chunk['next_type'] == -1:
    #         endpoint_text = ""
    #     else:
    #         print(chunk['next_type'])
    #         raise RuntimeError   
    #     return endpoint_text

    # def _generate_passed_text(self, chunk, presentation_type):
    #     """Generate human understandable text on what kind of hallways you passed
    #     """         

    #     passed_text = ""
    #     if "passed_hallways" in chunk and len(chunk['passed_hallways']) > 0:
    #         #remove the straight hallways
    #         chunk['passed_hallways'] = [x for x in chunk['passed_hallways'] if x != 1]

    #         #There are two ways to present these information
    #         #(1) - Count the number of hallways opennings
    #         if presentation_type == 'summary':
    #             left_count = 0
    #             right_count = 0
    #             both_count = 0
    #             for i,passed_hallways in enumerate(chunk['passed_hallways']):
    #                 if passed_hallways == 5:
    #                     left_count += 1
    #                 elif passed_hallways == 6:
    #                     right_count += 1
    #                 elif passed_hallways == 7:
    #                     both_count += 1
    #                 else:
    #                     print(passed_hallways)
    #                     raise RuntimeError

    #             text = "Along the way, you will pass,"
    #             if left_count > 0:
    #                 if left_count == 1:
    #                     text += " one openning to your left, "   
    #                 else:
    #                     text += " {} opennings to your left, ".format(left_count)            
    #             if right_count > 0:
    #                 if right_count == 1:
    #                     text += " one openning to your right, "   
    #                 else:
    #                     text += " {} opennings to your right, ".format(right_count)    
    #             if both_count > 0:
    #                 if both_count == 1:
    #                     text += " one openning on both sides, "   
    #                 else:
    #                     text += " {} opennings on both sides, ".format(both_count)     
    #             passed_text = text + "."


    #         #(2) Give a sequences of the openning directly
    #         elif presentation_type == 'individual':
    #             text = "Along the way, you will pass openings to your,"

    #             for i,passed_hallways in enumerate(chunk['passed_hallways']):
    #                 if passed_hallways == 5:
    #                     text += "left,"
    #                 elif passed_hallways == 6:text
    #                     text += "right,"
    #                 elif passed_hallways == 7:
    #                     text += "both sides,"
    #                 else:
    #                     print(passed_hallways)
    #                     raise RuntimeError
    #                 if i + 1 < len(chunk['passed_hallways']):
    #                     text += ' then '
    #             passed_text = text + "."
    #         else:
    #             print(presentation_type)
    #             raise RuntimeError     
    #     return passed_text   



    @staticmethod
    def generate_destination_text(chunk, name = ""):

        if chunk['clock'] == 0:
            side = "in front of you"
        elif chunk['clock'] == 3:
            side = "to the right of you"
        elif chunk['clock'] == 9:
            side = "to the left of you"
        else:
            side = "to the {}'o clock of you".format(int(chunk['clock']))
        
        flavor_text1 = "Your destination {} is ".format(name)
        flavor_text2 = "{}".format(side) 

        return flavor_text1, flavor_text2            