#!/usr/bin/env python

class Service(object):
    def __init__(self, name, timeout, success, datatype):
        self.name = name
        self.timeout = timeout
        self.success = success
        self.datatype = datatype
        self.available = True
        try:
            rospy.wait_for_service(name, timeout)
        except:
            self.available = False
            pass
        self.service_proxy = rospy.ServiceProxy(name, datatype)


    def call(self, *args, **kwargs):
        if self.available:
            try:
                res = self.service_proxy(*args)
                return self._evaluate_service(res, self.success)
            except rospy.ServiceException, e:
                self._service_warning(e)
                return False
            rospy.loginfo('Ran service %s', self.service.name)
        else:
            rospy.logwarn('Ran service %s, but was not available during init',
                          self.service.name)

    def _service_warning(self, exception):
        rospy.logwarn('Error encountered: %s', str(exception))

    def _evaluate_service(self, result, success):
        print 'res: ', result
        if str(result) == success:
            rospy.loginfo('Successfully ran service %s', self.service.name)
            return True
        else:
            rospy.loginfo('Error with service %s', self.service.name)
            return False
