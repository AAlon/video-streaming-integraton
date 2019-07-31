import sys
import time
import uuid
import subprocess as sp
import glob
import os
import shlex
import json
import massedit

REKOGNITION_POLICY_TEMPLATE = '''{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Action": [
                "sns:Publish"
            ],
            "Resource": "arn:aws:sns:*:*:AmazonRekognition*"
        },
        {
            "Effect": "Allow",
            "Action": [
                "kinesis:PutRecord",
                "kinesis:PutRecords"
            ],
            "Resource": "%(kds_arn)s"
        },
        {
            "Effect": "Allow",
            "Action": [
                "kinesisvideo:GetDataEndpoint",
                "kinesisvideo:GetMedia"
            ],
            "Resource": "%(kvs_arn)s"
        }
    ]
}'''

class KinesisTestManager(object):
    def __init__(self, test_run_id=uuid.uuid4().hex):
        self._verbose = False
        for arg in sys.argv[1:]:
            if arg == '-v' or arg == '--verbose':
                self.log('Verbose mode')
                self._verbose = True
            if os.path.exists(arg):
                # This is terrible.
                self._bag_path = arg

        self._run_id = test_run_id[:8] # Take first 8 characters
        self._collection_name = 'integ-test'
        self._bucket_name = 'rekog-test-bucket'
        self._region = 'us-west-2'
        self._data_stream_name = 'integ-test-ds'
        self._video_stream_name = 'integ-test-vs'
        self._stream_processor_name = 'integ-test-sp'
        self._rekognition_policy_name = 'integ-test-policy'
        self._rekognition_role_name = 'integ-test-role'
        self._encoder_subscription_topic = r'"/videofile/image_raw"'
        self._processes = []
        self._policy_doc_path = r'/tmp/policy_doc'
        #self._bag_path = 'new_format_short'

    def _fq(self, name):
        fully_qualified_name = self._run_id + '-' + name
        return fully_qualified_name

    def _update_configs(self):
        self.log('Updating configuration')
        self._streamer_prefix = sp.check_output('ros2 pkg prefix kinesis_video_streamer', shell=True).strip().decode('utf-8')
        streamer_config_path = os.path.join(self._streamer_prefix, 'share', 'kinesis_video_streamer', 'config', 'sample_config.yaml')
        self._encoder_prefix = sp.check_output('ros2 pkg prefix h264_video_encoder', shell=True).strip().decode('utf-8')
        encoder_config_path = os.path.join(self._encoder_prefix, 'share', 'h264_video_encoder', 'config', 'sample_configuration.yaml')

        massedit.edit_files(
          [streamer_config_path, ],
          ["re.sub('stream_name:(.*)', 'stream_name: %s', line)" % (self._fq(self._video_stream_name), ),
           "re.sub('rekognition_data_stream:(.*)', 'rekognition_data_stream: %s', line)" % (self._fq(self._data_stream_name), ),
           "re.sub('\#\srekognition_data_stream:', 'rekognition_data_stream:', line)",
           "re.sub('topic_type:(.*)', 'topic_type: 3', line)",
           "re.sub('rekognition_topic_name:(.*)', 'rekognition_topic_name: /rekognition/results', line)",
           "re.sub('\#\srekognition_topic_name:', 'rekognition_topic_name:', line)"
          ],
        dry_run=False)

        massedit.edit_files([encoder_config_path, ],
          ["re.sub('subscription_topic:(.*)', 'subscription_topic: %s', line)" % (self._encoder_subscription_topic, ), ],
        dry_run=False)

    def setup(self):
        # Create Rekognition collection
        create_collection_cmd = '''aws rekognition create-collection --collection-id %(collection_name)s  --region %(region)s''' % {
            'collection_name': self._fq(self._collection_name),
            'region': self._region
        }
        sp.check_output(create_collection_cmd, shell=True)
        self.log('Created collection %s' % (self._fq(self._collection_name), ))

        # Create S3 bucket
        create_bucket_cmd = '''aws s3 mb s3://%(bucket_name)s''' % {
            'bucket_name': self._fq(self._bucket_name)
        }
        sp.check_output(create_bucket_cmd, shell=True)
        self.log('Created bucket %s' % (self._fq(self._bucket_name), ))

        face_files = glob.glob(os.path.join('assets', 'faces', '*'))
        # Upload face files to s3
        for face_file in face_files:
            aws_s3_copy = 'aws s3 cp %(local_path)s s3://%(bucket_name)s/%(remote_path)s' % {
                'local_path': face_file,
                'bucket_name': self._fq(self._bucket_name),
                'remote_path': face_file
            }
            self.log('Uploading %s' % (face_file, ))
            sp.check_output(aws_s3_copy, shell=True)

        # Prepare human readable file names: 'jeff-bezos.jpeg' -> 'jeff bezos'
        faces_to_names = {}
        for file_name in face_files:
            person = os.path.basename(file_name).split('.jp')[0]
            faces_to_names[file_name] = person
        self._faces = faces_to_names

        # Index faces
        for face_file, person in faces_to_names.items():
            index_faces_cmd = '''aws rekognition index-faces --image '{"S3Object":{"Bucket":"%(bucket_name)s","Name":"%(face_file)s"}}' --collection-id "%(collection_name)s" --detection-attributes "ALL" --external-image-id "%(face_name)s" --region %(region)s''' % {
                'bucket_name': self._fq(self._bucket_name),
                'face_file': face_file,
                'collection_name': self._fq(self._collection_name),
                'face_name': person,
                'region': self._region,
            }
            self.log('Indexing face: %s' % (face_file, ))
            sp.check_output(index_faces_cmd, shell=True)

        # Data stream
        kinesis_create_stream = '''aws kinesis create-stream --stream-name %(data_stream_name)s --shard-count 4''' % {
            'data_stream_name': self._fq(self._data_stream_name)
        }
        sp.check_output(kinesis_create_stream, shell=True)
        kinesis_get_arn = '''aws kinesis describe-stream --stream-name %(data_stream_name)s | jq -r ."StreamDescription"."StreamARN"''' % {
            'data_stream_name': self._fq(self._data_stream_name)
        }
        self._kinesis_data_stream_arn = sp.check_output(kinesis_get_arn, shell=True).strip().decode('utf-8')

        # Video stream
        self._kinesis_video_stream_arn = sp.check_output('''aws kinesisvideo create-stream --stream-name %(video_stream_name)s --data-retention-in-hours 2 | jq -r ."StreamARN"''' % {
           'video_stream_name': self._fq(self._video_stream_name)
        }, shell=True).strip().decode('utf-8')

        stream_status = 'CREATING'
        while stream_status != 'ACTIVE':
            time.sleep(1)
            self.log('Waiting for data stream to be created... status: %s' % (stream_status, ))
            stream_status = sp.check_output('aws kinesis describe-stream --stream-name %s | jq -r ."StreamDescription"."StreamStatus"' % (self._fq(self._data_stream_name), ), shell=True).strip().decode('utf-8')

        # Role
        rekognition_policy = REKOGNITION_POLICY_TEMPLATE % {'kvs_arn': self._kinesis_video_stream_arn, 'kds_arn': self._kinesis_data_stream_arn}
        with open(self._policy_doc_path, 'w') as f:
            f.write(rekognition_policy)

        self._rekognition_policy_arn = sp.check_output('''aws iam create-policy --policy-name %(policy_name)s --policy-document file://%(policy_doc)s | jq -r ."Policy"."Arn"''' % {
            'policy_name': self._fq(self._rekognition_policy_name),
            'policy_doc': self._policy_doc_path
        }, shell=True).strip().decode('utf-8')

        self._rekognition_role_arn = sp.check_output('''aws iam create-role --role-name %(test_role)s --assume-role-policy-document file://assets/trust_rekognition | jq -r ."Role"."Arn"''' % {
            'test_role': self._fq(self._rekognition_role_name)
        }, shell=True).strip().decode('utf-8')
        self.log('Waiting for IAM resources to become available')
        time.sleep(10)

        sp.check_output('''aws iam attach-role-policy --policy-arn %(policy_arn)s --role-name %(role_name)s''' % {
            'policy_arn': self._rekognition_policy_arn,
            'role_name': self._fq(self._rekognition_role_name)
        }, shell=True)
        time.sleep(10)

        # Stream processor
        self.log('Creating stream processor')
        create_stream_processor = '''aws rekognition create-stream-processor --name %(stream_processor)s --input KinesisVideoStream={Arn=%(kvs_arn)s} --settings FaceSearch={CollectionId=%(collection_name)s,FaceMatchThreshold=75.0} --role-arn %(role_arn)s --stream-processor-output KinesisDataStream={Arn=%(kds_arn)s}''' % {
            'kvs_arn': self._kinesis_video_stream_arn,
            'kds_arn': self._kinesis_data_stream_arn,
            'stream_processor': self._fq(self._stream_processor_name),
            'collection_name': self._fq(self._collection_name),
            'role_arn': self._rekognition_role_arn
        }
        sp.check_output(create_stream_processor, shell=True)
        self.log('Starting stream processor')
        start_stream_processor = 'aws rekognition start-stream-processor --name %s' % (self._fq(self._stream_processor_name), )
        sp.check_output(start_stream_processor, shell=True)

        self.log('Waiting 5 seconds in case of propagation delay')
        time.sleep(5)

    def teardown(self):
        time.sleep(1)
        # Delete KVS
        sp.check_output('aws kinesisvideo delete-stream --stream-arn %s' % (self._kinesis_video_stream_arn, ), shell=True)
        # Delete KDS
        sp.check_output('aws kinesis delete-stream --stream-name %s' % (self._fq(self._data_stream_name), ), shell=True)
        # Stream processor
        time.sleep(5)
        sp.check_output('aws rekognition stop-stream-processor --name %s' % (self._fq(self._stream_processor_name), ), shell=True)
        sp.check_output('aws rekognition delete-stream-processor --name %s' % (self._fq(self._stream_processor_name), ), shell=True)
        # Detach policy from role
        sp.check_output('aws iam detach-role-policy --role-name %s --policy-arn %s' % (self._fq(self._rekognition_role_name), self._rekognition_policy_arn, ), shell=True)
        # Role
        sp.check_output('aws iam delete-role --role-name %s' % (self._fq(self._rekognition_role_name), ), shell=True)
        # Policy
        sp.check_output('aws iam delete-policy --policy-arn %s' % (self._rekognition_policy_arn, ), shell=True)
        # Collection
        delete_collection_cmd = '''aws rekognition delete-collection --collection-id %(collection_name)s''' % {
          'collection_name': self._fq(self._collection_name)
        }
        sp.check_output(delete_collection_cmd, shell=True)
        # Bucket
        sp.check_output('aws s3 rb s3://%s --force' % (self._fq(self._bucket_name), ), shell=True)

        self.log('Finished cleaning up resources')

    def log(self, msg):
        print(msg)

    def _assert_faces_in_file(self, count):
        names = self._faces.values()
        with open(self._rekognition_results_file, 'r') as f:
            lines = f.readlines()
        matches = 0
        for line in lines:
            for name in names:
                if name in line:
                    matches += 1
                    break

        self.log('Verifying that at least %d matches were found...' % (count, ))
        assert matches >= count
        self.log('* PASS')

    def test_end_to_end_rekognition(self):
        self.log('Starting up streamer, encoder, and rosbag player')
        # Get the rosbag & extract
        self.start_up_core_nodes()
        # ros2 bag play
        time.sleep(2)
        self.rosbag_play()
        time.sleep(2)

        self._rekognition_results_file = r'/tmp/rekognition_results'
        if os.path.isfile(self._rekognition_results_file):
            os.remove(self._rekognition_results_file)
        echo_cmd = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'topic_to_file.sh') + ' /rekognition/results std_msgs/msg/String ' + self._rekognition_results_file
        topic_echo = sp.Popen(shlex.split(echo_cmd), preexec_fn=os.setsid)
        self._processes.append(topic_echo)

        self.log('Running for 60 seconds')
        time.sleep(65)
        self._assert_faces_in_file(5)

        self.log('Finished %s' % (self.test_end_to_end_rekognition.__name__, ))
        self.kill_nodes()

    def rosbag_play(self):
        bag_play_cmd = 'ros2 bag play %s' % (self._bag_path, )
        bag_play = sp.Popen(shlex.split(bag_play_cmd), preexec_fn=os.setsid)
        self._processes.append(bag_play)

    def start_up_core_nodes(self):
        # Run encoder
        encoder_cmd = 'ros2 launch h264_video_encoder h264_video_encoder_launch.py --screen'
        if self._verbose:
            encoder = sp.Popen(shlex.split(encoder_cmd), preexec_fn=os.setsid)
        else:
            encoder = sp.Popen(shlex.split(encoder_cmd), stdout=sp.DEVNULL, preexec_fn=os.setsid)

        # Run streamer
        streamer_cmd = 'ros2 launch kinesis_video_streamer kinesis_video_streamer.launch.py --screen'
        if self._verbose:
            streamer = sp.Popen(shlex.split(streamer_cmd), preexec_fn=os.setsid)
        else:
            streamer = sp.Popen(shlex.split(streamer_cmd), stdout=sp.DEVNULL, preexec_fn=os.setsid)

        self._processes.append(encoder)
        self._processes.append(streamer)

        time.sleep(1)

    def kill_nodes(self):
       self.log('Killing all nodes')
       for p in self._processes:
           p.kill()
           os.killpg(os.getpgid(p.pid), 15)

    def run(self):
        exit_status = 0
        self.log('Setting up')
        self.setup()
        self._update_configs()
        try:
            self.test_end_to_end_rekognition()
        except Exception as e:
            exit_status = 1
            self.log(e)
            self.kill_nodes()

        self.log('Tearing down')
        self.teardown()
        return exit_status


def main():
    test_manager = KinesisTestManager()
    sys.exit(test_manager.run())

if __name__ == '__main__':
    main()
