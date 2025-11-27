
import rclpy
from rclpy.node import Node
import pickle
import numpy as np
import os


from feature_extractor_pkg.srv import Classification

class ModelServiceNode(Node):
    def __init__(self):
        super().__init__('model_classifier_node')

        self.declare_parameter('model_path', '/home/akhil/panda_arm/best_model.sav')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.model = None
        self.scaler = None
        self.encoder = None

        self.load_model(model_path)

        self.srv = self.create_service(
            Classification,
            'model_classification_service',
            self.classification_callback(Classification.Request, Classification.Response)
        )
        self.get_logger().info('Service Server Ready: waiting for requests...')

    def load_model(self, path):
        try:
            if not os.path.exists(path):
                self.get_logger().error(f"Model file not found at: {path}")
                return

            self.get_logger().info(f"Loading model bundle from {path}...")
            
            with open(path, 'rb') as f:
                bundle = pickle.load(f)
            
            self.model = bundle['model']
            self.scaler = bundle['scaler']
            self.encoder = bundle['encoder']
            
            self.get_logger().info("Model, Scaler, and Encoder loaded successfully.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def classification_callback(self, request, response):
        if self.model is None:
            self.get_logger().error("Cannot predict: Model not loaded.")
            response.class_label = "ERROR_NO_MODEL"
            response.confidence_score = 0.0
            return response

        self.get_logger().info(f"Received request with {len(request.input_features)} features.")

        try:
            
            input_data = np.array(request.input_features).reshape(1, -1)

    
            scaled_data = self.scaler.transform(input_data)

            prediction_encoded = self.model.predict(scaled_data)
            confidence_scores = np.max(self.model.predict_proba(scaled_data))

            prediction_label = self.encoder.inverse_transform(prediction_encoded)

            response.class_label = str(prediction_label[0])
            response.confidence_score = float(confidence_scores)
            self.get_logger().info(f"Prediction: {response.class_label}")

        except Exception as e:
            self.get_logger().error(f"Prediction logic failed: {e}")
            response.label = "ERROR_COMPUTATION"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ModelServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()