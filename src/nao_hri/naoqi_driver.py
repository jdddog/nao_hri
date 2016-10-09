# Copyright 2009-2011 Armin Hornung, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from threading import Thread

import rospy
from optparse import OptionParser

# import Aldebaran API (must be in PYTHONPATH):
try:
    from naoqi import ALProxy, ALBroker, ALModule
except ImportError:
    msg = "Error importing NaoQI. Please make sure that Aldebaran's NaoQI API is in your PYTHONPATH."
    rospy.logerr(msg)
    raise RuntimeError(msg)


class NaoNode(ALModule):

    def __init__(self, module_name):
        self.parse_options(module_name)
        self.init_broker()

        # A distutils.version.LooseVersion that contains the current verssion of NAOqi we're connected to
        self.__naoqi_version = None

        ## NAOqi stuff
        # dict from a modulename to a proxy
        self.__proxies = {}

        # If user has set parameters for ip and port use them as default
        self.default_ip = rospy.get_param("~pip", "127.0.0.1")
        self.default_port = rospy.get_param("~pport", 9559)

    def parse_options(self, module_name):
        parser = OptionParser()
        parser.add_option("--ip", dest = "ip", default = "",
                          help = "IP/hostname of broker. Default is system's default IP address.", metavar = "IP")
        parser.add_option("--port", dest = "port", default = 0,
                          help = "IP/hostname of broker. Default is 1051.", metavar = "PORT")
        parser.add_option("--pip", dest = "pip", default = "127.0.0.1",
                          help = "IP/hostname of parent broker. Default is 127.0.0.1.", metavar = "IP")
        parser.add_option("--pport", dest = "pport", default = 9559,
                          help = "port of parent broker. Default is 9559.", metavar = "PORT")

        (options, args) = parser.parse_args()
        self.ip = options.ip
        self.port = int(options.port)
        self.pip = options.pip
        self.pport = int(options.pport)
        self.module_name = module_name

    def init_broker(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.module_name, self.ip, self.port, self.pip, self.pport)
        except RuntimeError, e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.module_name)

        self.memProxy = ALProxy("ALMemory", self.pip, self.pport)
        # TODO: check self.memProxy.version() for > 1.6
        if self.memProxy is None:
            rospy.logerr("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)

    def get_proxy(self, name, warn=True):
        """
        Returns a proxy to a specific module. If it has not been created yet, it is created
        :param name: the name of the module to create a proxy for
        :return: a proxy to the corresponding module
        """
        if name in self.__proxies and self.__proxies[name] is not None:
            return self.__proxies[name]

        proxy = None
        try:
            proxy = ALProxy(name, self.pip, self.pport)
        except RuntimeError,e:
            if warn:
                rospy.logerr("Could not create Proxy to \"%s\". \nException message:\n%s",name, e)

        self.__proxies[name] = proxy
        return proxy

    def get_version(self):
        """
        Returns the NAOqi version.
        A proxy for ALMemory is automatically created if needed as self.memProxy.
        You can then simply have code that runs or not depending on the NAOqi version.
        E.g. if distutils.version.LooseVersion('1.6') < get_version()    ....
        :return: a distutils.version.LooseVersion object with the NAOqi version
        """
        if self.__naoqi_version is None:
            proxy = self.get_proxy('ALMemory')
            if proxy is None:
                # exiting is bad but it should not happen
                # except maybe with NAOqi versions < 1.6 or future ones
                # in which case we will adapt that code to call the proper
                # version function
                exit(1)

            from distutils.version import LooseVersion
            self.__naoqi_version = LooseVersion(proxy.version())

        return self.__naoqi_version

    # A stupid hack:
    #   Instead of taking a function pointer as a callback (e.g. self.word_pos_changed)
    #   subscribeToEvent / unsubscribeToEvent take the name of the class instance as a string and the name of the method
    #   as a string.
    #
    #   Hence, this method finds it for you automatically (if its in the same file)
    #
    #   pass it globals() from the outer scope

    def get_instance_name(self, global_variables):
        instance_name = ''

        for k, v in list(global_variables.iteritems()):
            if v is self:
                instance_name = k

        return instance_name