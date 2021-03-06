# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

from __future__ import unicode_literals

import hashlib
import os
import unittest
import shutil
import sys
import tempfile

from mozfile.mozfile import NamedTemporaryFile
from mozunit import (
    main,
    MockedOpen,
)

from mozbuild.util import (
    FileAvoidWrite,
    hash_file,
    memoize,
    memoized_property,
    resolve_target_to_make,
    MozbuildDeletionError,
    HierarchicalStringList,
    HierarchicalStringListWithFlagsFactory,
    StrictOrderingOnAppendList,
    StrictOrderingOnAppendListWithFlagsFactory,
    UnsortedError,
)

if sys.version_info[0] == 3:
    str_type = 'str'
else:
    str_type = 'unicode'

data_path = os.path.abspath(os.path.dirname(__file__))
data_path = os.path.join(data_path, 'data')


class TestHashing(unittest.TestCase):
    def test_hash_file_known_hash(self):
        """Ensure a known hash value is recreated."""
        data = b'The quick brown fox jumps over the lazy cog'
        expected = 'de9f2c7fd25e1b3afad3e85a0bd17d9b100db4b3'

        temp = NamedTemporaryFile()
        temp.write(data)
        temp.flush()

        actual = hash_file(temp.name)

        self.assertEqual(actual, expected)

    def test_hash_file_large(self):
        """Ensure that hash_file seems to work with a large file."""
        data = b'x' * 1048576

        hasher = hashlib.sha1()
        hasher.update(data)
        expected = hasher.hexdigest()

        temp = NamedTemporaryFile()
        temp.write(data)
        temp.flush()

        actual = hash_file(temp.name)

        self.assertEqual(actual, expected)


class TestFileAvoidWrite(unittest.TestCase):
    def test_file_avoid_write(self):
        with MockedOpen({'file': 'content'}):
            # Overwriting an existing file replaces its content
            faw = FileAvoidWrite('file')
            faw.write('bazqux')
            self.assertEqual(faw.close(), (True, True))
            self.assertEqual(open('file', 'r').read(), 'bazqux')

            # Creating a new file (obviously) stores its content
            faw = FileAvoidWrite('file2')
            faw.write('content')
            self.assertEqual(faw.close(), (False, True))
            self.assertEqual(open('file2').read(), 'content')

        with MockedOpen({'file': 'content'}):
            with FileAvoidWrite('file') as file:
                file.write('foobar')

            self.assertEqual(open('file', 'r').read(), 'foobar')

        class MyMockedOpen(MockedOpen):
            '''MockedOpen extension to raise an exception if something
            attempts to write in an opened file.
            '''
            def __call__(self, name, mode):
                if 'w' in mode:
                    raise Exception, 'Unexpected open with write mode'
                return MockedOpen.__call__(self, name, mode)

        with MyMockedOpen({'file': 'content'}):
            # Validate that MyMockedOpen works as intended
            file = FileAvoidWrite('file')
            file.write('foobar')
            self.assertRaises(Exception, file.close)

            # Check that no write actually happens when writing the
            # same content as what already is in the file
            faw = FileAvoidWrite('file')
            faw.write('content')
            self.assertEqual(faw.close(), (True, False))

    def test_diff_not_default(self):
        """Diffs are not produced by default."""

        with MockedOpen({'file': 'old'}):
            faw = FileAvoidWrite('file')
            faw.write('dummy')
            faw.close()
            self.assertIsNone(faw.diff)

    def test_diff_update(self):
        """Diffs are produced on file update."""

        with MockedOpen({'file': 'old'}):
            faw = FileAvoidWrite('file', capture_diff=True)
            faw.write('new')
            faw.close()

            diff = '\n'.join(faw.diff)
            self.assertIn('-old', diff)
            self.assertIn('+new', diff)

    def test_diff_create(self):
        """Diffs are produced when files are created."""

        tmpdir = tempfile.mkdtemp()
        try:
            path = os.path.join(tmpdir, 'file')
            faw = FileAvoidWrite(path, capture_diff=True)
            faw.write('new')
            faw.close()

            diff = '\n'.join(faw.diff)
            self.assertIn('+new', diff)
        finally:
            shutil.rmtree(tmpdir)

class TestResolveTargetToMake(unittest.TestCase):
    def setUp(self):
        self.topobjdir = data_path

    def assertResolve(self, path, expected):
        # Handle Windows path separators.
        (reldir, target) = resolve_target_to_make(self.topobjdir, path)
        if reldir is not None:
            reldir = reldir.replace(os.sep, '/')
        if target is not None:
            target = target.replace(os.sep, '/')
        self.assertEqual((reldir, target), expected)

    def test_root_path(self):
        self.assertResolve('/test-dir', ('test-dir', None))
        self.assertResolve('/test-dir/with', ('test-dir/with', None))
        self.assertResolve('/test-dir/without', ('test-dir', None))
        self.assertResolve('/test-dir/without/with', ('test-dir/without/with', None))

    def test_dir(self):
        self.assertResolve('test-dir', ('test-dir', None))
        self.assertResolve('test-dir/with', ('test-dir/with', None))
        self.assertResolve('test-dir/with', ('test-dir/with', None))
        self.assertResolve('test-dir/without', ('test-dir', None))
        self.assertResolve('test-dir/without/with', ('test-dir/without/with', None))

    def test_top_level(self):
        self.assertResolve('package', (None, 'package'))
        # Makefile handling shouldn't affect top-level targets.
        self.assertResolve('Makefile', (None, 'Makefile'))

    def test_regular_file(self):
        self.assertResolve('test-dir/with/file', ('test-dir/with', 'file'))
        self.assertResolve('test-dir/with/without/file', ('test-dir/with', 'without/file'))
        self.assertResolve('test-dir/with/without/with/file', ('test-dir/with/without/with', 'file'))

        self.assertResolve('test-dir/without/file', ('test-dir', 'without/file'))
        self.assertResolve('test-dir/without/with/file', ('test-dir/without/with', 'file'))
        self.assertResolve('test-dir/without/with/without/file', ('test-dir/without/with', 'without/file'))

    def test_Makefile(self):
        self.assertResolve('test-dir/with/Makefile', ('test-dir', 'with/Makefile'))
        self.assertResolve('test-dir/with/without/Makefile', ('test-dir/with', 'without/Makefile'))
        self.assertResolve('test-dir/with/without/with/Makefile', ('test-dir/with', 'without/with/Makefile'))

        self.assertResolve('test-dir/without/Makefile', ('test-dir', 'without/Makefile'))
        self.assertResolve('test-dir/without/with/Makefile', ('test-dir', 'without/with/Makefile'))
        self.assertResolve('test-dir/without/with/without/Makefile', ('test-dir/without/with', 'without/Makefile'))

class TestHierarchicalStringList(unittest.TestCase):
    def setUp(self):
        self.EXPORTS = HierarchicalStringList()

    def test_exports_append(self):
        self.assertEqual(self.EXPORTS._strings, [])
        self.EXPORTS += ["foo.h"]
        self.assertEqual(self.EXPORTS._strings, ["foo.h"])
        self.EXPORTS += ["bar.h"]
        self.assertEqual(self.EXPORTS._strings, ["foo.h", "bar.h"])

    def test_exports_subdir(self):
        self.assertEqual(self.EXPORTS._children, {})
        self.EXPORTS.foo += ["foo.h"]
        self.assertItemsEqual(self.EXPORTS._children, {"foo" : True})
        self.assertEqual(self.EXPORTS.foo._strings, ["foo.h"])
        self.EXPORTS.bar += ["bar.h"]
        self.assertItemsEqual(self.EXPORTS._children,
                              {"foo" : True, "bar" : True})
        self.assertEqual(self.EXPORTS.foo._strings, ["foo.h"])
        self.assertEqual(self.EXPORTS.bar._strings, ["bar.h"])

    def test_exports_multiple_subdir(self):
        self.EXPORTS.foo.bar = ["foobar.h"]
        self.assertItemsEqual(self.EXPORTS._children, {"foo" : True})
        self.assertItemsEqual(self.EXPORTS.foo._children, {"bar" : True})
        self.assertItemsEqual(self.EXPORTS.foo.bar._children, {})
        self.assertEqual(self.EXPORTS._strings, [])
        self.assertEqual(self.EXPORTS.foo._strings, [])
        self.assertEqual(self.EXPORTS.foo.bar._strings, ["foobar.h"])

    def test_invalid_exports_append(self):
        with self.assertRaises(ValueError) as ve:
            self.EXPORTS += "foo.h"
        self.assertEqual(str(ve.exception),
                         "Expected a list of strings, not <type '%s'>" % str_type)

    def test_invalid_exports_set(self):
        with self.assertRaises(ValueError) as ve:
            self.EXPORTS.foo = "foo.h"

        self.assertEqual(str(ve.exception),
                         "Expected a list of strings, not <type '%s'>" % str_type)

    def test_invalid_exports_append_base(self):
        with self.assertRaises(ValueError) as ve:
            self.EXPORTS += "foo.h"

        self.assertEqual(str(ve.exception),
                         "Expected a list of strings, not <type '%s'>" % str_type)

    def test_invalid_exports_bool(self):
        with self.assertRaises(ValueError) as ve:
            self.EXPORTS += [True]

        self.assertEqual(str(ve.exception),
                         "Expected a list of strings, not an element of "
                         "<type 'bool'>")

    def test_del_exports(self):
        with self.assertRaises(MozbuildDeletionError) as mde:
            self.EXPORTS.foo += ['bar.h']
            del self.EXPORTS.foo

    def test_unsorted_appends(self):
        with self.assertRaises(UnsortedError) as ee:
            self.EXPORTS += ['foo.h', 'bar.h']

    def test_walk(self):
        l = HierarchicalStringList()
        l += ['root1', 'root2', 'root3']
        l.child1 += ['child11', 'child12', 'child13']
        l.child1.grandchild1 += ['grandchild111', 'grandchild112']
        l.child1.grandchild2 += ['grandchild121', 'grandchild122']
        l.child2.grandchild1 += ['grandchild211', 'grandchild212']
        l.child2.grandchild1 += ['grandchild213', 'grandchild214']

        els = list((path, list(seq)) for path, seq in l.walk())
        self.assertEqual(els, [
            ('', ['root1', 'root2', 'root3']),
            ('child1', ['child11', 'child12', 'child13']),
            ('child1/grandchild1', ['grandchild111', 'grandchild112']),
            ('child1/grandchild2', ['grandchild121', 'grandchild122']),
            ('child2/grandchild1', ['grandchild211', 'grandchild212',
                                    'grandchild213', 'grandchild214']),
        ])


class TestStrictOrderingOnAppendList(unittest.TestCase):
    def test_init(self):
        l = StrictOrderingOnAppendList()
        self.assertEqual(len(l), 0)

        l = StrictOrderingOnAppendList(['a', 'b', 'c'])
        self.assertEqual(len(l), 3)

        with self.assertRaises(UnsortedError):
            StrictOrderingOnAppendList(['c', 'b', 'a'])

        self.assertEqual(len(l), 3)

    def test_extend(self):
        l = StrictOrderingOnAppendList()
        l.extend(['a', 'b'])
        self.assertEqual(len(l), 2)
        self.assertIsInstance(l, StrictOrderingOnAppendList)

        with self.assertRaises(UnsortedError):
            l.extend(['d', 'c'])

        self.assertEqual(len(l), 2)

    def test_slicing(self):
        l = StrictOrderingOnAppendList()
        l[:] = ['a', 'b']
        self.assertEqual(len(l), 2)
        self.assertIsInstance(l, StrictOrderingOnAppendList)

        with self.assertRaises(UnsortedError):
            l[:] = ['b', 'a']

        self.assertEqual(len(l), 2)

    def test_add(self):
        l = StrictOrderingOnAppendList()
        l2 = l + ['a', 'b']
        self.assertEqual(len(l), 0)
        self.assertEqual(len(l2), 2)
        self.assertIsInstance(l2, StrictOrderingOnAppendList)

        with self.assertRaises(UnsortedError):
            l2 = l + ['b', 'a']

        self.assertEqual(len(l), 0)

    def test_iadd(self):
        l = StrictOrderingOnAppendList()
        l += ['a', 'b']
        self.assertEqual(len(l), 2)
        self.assertIsInstance(l, StrictOrderingOnAppendList)

        with self.assertRaises(UnsortedError):
            l += ['b', 'a']

        self.assertEqual(len(l), 2)

    def test_add_after_iadd(self):
        l = StrictOrderingOnAppendList(['b'])
        l += ['a']
        l2 = l + ['c', 'd']
        self.assertEqual(len(l), 2)
        self.assertEqual(len(l2), 4)
        self.assertIsInstance(l2, StrictOrderingOnAppendList)
        with self.assertRaises(UnsortedError):
            l2 = l + ['d', 'c']

        self.assertEqual(len(l), 2)

    def test_add_StrictOrderingOnAppendList(self):
        l = StrictOrderingOnAppendList()
        l += ['c', 'd']
        l += ['a', 'b']
        l2 = StrictOrderingOnAppendList()
        with self.assertRaises(UnsortedError):
            l2 += list(l)
        # Adding a StrictOrderingOnAppendList to another shouldn't throw
        l2 += l


class TestStrictOrderingOnAppendListWithFlagsFactory(unittest.TestCase):
    def test_strict_ordering_on_append_list_with_flags_factory(self):
        cls = StrictOrderingOnAppendListWithFlagsFactory({
            'foo': bool,
            'bar': int,
        })

        l = cls()
        l += ['a', 'b']

        with self.assertRaises(Exception):
            l['a'] = 'foo'

        with self.assertRaises(Exception):
            c = l['c']

        self.assertEqual(l['a'].foo, False)
        l['a'].foo = True
        self.assertEqual(l['a'].foo, True)

        with self.assertRaises(TypeError):
            l['a'].bar = 'bar'

        self.assertEqual(l['a'].bar, 0)
        l['a'].bar = 42
        self.assertEqual(l['a'].bar, 42)

        l['b'].foo = True
        self.assertEqual(l['b'].foo, True)

        with self.assertRaises(AttributeError):
            l['b'].baz = False

        l['b'].update(foo=False, bar=12)
        self.assertEqual(l['b'].foo, False)
        self.assertEqual(l['b'].bar, 12)

        with self.assertRaises(AttributeError):
            l['b'].update(xyz=1)


class TestHierarchicalStringListWithFlagsFactory(unittest.TestCase):
    def test_hierarchical_string_list_with_flags_factory(self):
        cls = HierarchicalStringListWithFlagsFactory({
            'foo': bool,
            'bar': int,
        })

        l = cls()
        l += ['a', 'b']

        with self.assertRaises(Exception):
            l['a'] = 'foo'

        with self.assertRaises(Exception):
            c = l['c']

        self.assertEqual(l['a'].foo, False)
        l['a'].foo = True
        self.assertEqual(l['a'].foo, True)

        with self.assertRaises(TypeError):
            l['a'].bar = 'bar'

        self.assertEqual(l['a'].bar, 0)
        l['a'].bar = 42
        self.assertEqual(l['a'].bar, 42)

        l['b'].foo = True
        self.assertEqual(l['b'].foo, True)

        with self.assertRaises(AttributeError):
            l['b'].baz = False

        l.x += ['x', 'y']

        with self.assertRaises(Exception):
            l.x['x'] = 'foo'

        with self.assertRaises(Exception):
            c = l.x['c']

        self.assertEqual(l.x['x'].foo, False)
        l.x['x'].foo = True
        self.assertEqual(l.x['x'].foo, True)

        with self.assertRaises(TypeError):
            l.x['x'].bar = 'bar'

        self.assertEqual(l.x['x'].bar, 0)
        l.x['x'].bar = 42
        self.assertEqual(l.x['x'].bar, 42)

        l.x['y'].foo = True
        self.assertEqual(l.x['y'].foo, True)

        with self.assertRaises(AttributeError):
            l.x['y'].baz = False


class TestMemoize(unittest.TestCase):
    def test_memoize(self):
        self._count = 0
        @memoize
        def wrapped(a, b):
            self._count += 1
            return a + b

        self.assertEqual(self._count, 0)
        self.assertEqual(wrapped(1, 1), 2)
        self.assertEqual(self._count, 1)
        self.assertEqual(wrapped(1, 1), 2)
        self.assertEqual(self._count, 1)
        self.assertEqual(wrapped(2, 1), 3)
        self.assertEqual(self._count, 2)
        self.assertEqual(wrapped(1, 2), 3)
        self.assertEqual(self._count, 3)
        self.assertEqual(wrapped(1, 2), 3)
        self.assertEqual(self._count, 3)
        self.assertEqual(wrapped(1, 1), 2)
        self.assertEqual(self._count, 3)

    def test_memoize_method(self):
        class foo(object):
            def __init__(self):
                self._count = 0

            @memoize
            def wrapped(self, a, b):
                self._count += 1
                return a + b

        instance = foo()
        refcount = sys.getrefcount(instance)
        self.assertEqual(instance._count, 0)
        self.assertEqual(instance.wrapped(1, 1), 2)
        self.assertEqual(instance._count, 1)
        self.assertEqual(instance.wrapped(1, 1), 2)
        self.assertEqual(instance._count, 1)
        self.assertEqual(instance.wrapped(2, 1), 3)
        self.assertEqual(instance._count, 2)
        self.assertEqual(instance.wrapped(1, 2), 3)
        self.assertEqual(instance._count, 3)
        self.assertEqual(instance.wrapped(1, 2), 3)
        self.assertEqual(instance._count, 3)
        self.assertEqual(instance.wrapped(1, 1), 2)
        self.assertEqual(instance._count, 3)

        # Memoization of methods is expected to not keep references to
        # instances, so the refcount shouldn't have changed after executing the
        # memoized method.
        self.assertEqual(refcount, sys.getrefcount(instance))

    def test_memoized_property(self):
        class foo(object):
            def __init__(self):
                self._count = 0

            @memoized_property
            def wrapped(self):
                self._count += 1
                return 42

        instance = foo()
        self.assertEqual(instance._count, 0)
        self.assertEqual(instance.wrapped, 42)
        self.assertEqual(instance._count, 1)
        self.assertEqual(instance.wrapped, 42)
        self.assertEqual(instance._count, 1)

if __name__ == '__main__':
    main()
