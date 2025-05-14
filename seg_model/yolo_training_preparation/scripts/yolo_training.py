from ultralytics import YOLO
import yaml
import os

class YoloTraining:
    
    def __init__(self):
        
        ''' Get the pkg path '''
        self.pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        
        ''' load config - dataset coco'''
        self.yolo_config_path = os.path.join(self.pkg_path, 'config/yolo_training_param.yaml')
        
        ''' load config for training model'''
        self.params_config = os.path.join(self.pkg_path, 'config/params.yaml')
        
        ''' load the model '''
        self.model = YOLO(self.params_config['yolo_training']['model'])

        ''' train the model '''
        self.model.train(data= self.yolo_config_path, 
                         epochs=self.params_config['yolo_training']['epochs'], 
                         imgsz=self.params_config['yolo_training']['imgsz'], 
                         batch=self.params_config['yolo_training']['batch_size'])

    def load_config(self):
        '''load config file'''
        config_file = self.pkg_path + '/config/yolo_training_param.yaml'
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            print('Configuration file not found: %s', config_file)
        return {}  
    
def main():
    try:
        node = YoloTraining()
        print("Training Started successfully!")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
