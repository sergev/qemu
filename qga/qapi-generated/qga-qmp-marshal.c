/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * schema-defined QMP->QAPI command dispatch
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "qemu-common.h"
#include "qemu/module.h"
#include "qapi/qmp/types.h"
#include "qapi/qmp/dispatch.h"
#include "qapi/visitor.h"
#include "qapi/qmp-output-visitor.h"
#include "qapi/qmp-input-visitor.h"
#include "qapi/dealloc-visitor.h"
#include "qga-qapi-types.h"
#include "qga-qapi-visit.h"
#include "qga-qmp-commands.h"

static void qmp_marshal_output_guest_sync_delimited(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_sync_delimited(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t id = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_guest_sync_delimited(id, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_sync_delimited(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_sync(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_sync(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t id = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_guest_sync(id, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_sync(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_ping(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_guest_ping(&local_err);
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_get_time(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_get_time(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;

    (void)args;
    retval = qmp_guest_get_time(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_get_time(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_input_guest_set_time(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_time = false;
    int64_t time = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_time, "time", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_time) {
        visit_type_int(v, &time, "time", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_guest_set_time(has_time, time, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_time, "time", NULL);
    if (has_time) {
        visit_type_int(v, &time, "time", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_info(GuestAgentInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestAgentInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestAgentInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_info(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestAgentInfo *retval = NULL;

    (void)args;
    retval = qmp_guest_info(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_info(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_input_guest_shutdown(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_mode = false;
    char *mode = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_mode, "mode", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_mode) {
        visit_type_str(v, &mode, "mode", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_guest_shutdown(has_mode, mode, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_mode, "mode", NULL);
    if (has_mode) {
        visit_type_str(v, &mode, "mode", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_file_open(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_file_open(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *path = NULL;
    bool has_mode = false;
    char *mode = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &path, "path", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_mode, "mode", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_mode) {
        visit_type_str(v, &mode, "mode", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_guest_file_open(path, has_mode, mode, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_file_open(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &path, "path", NULL);
    visit_optional(v, &has_mode, "mode", NULL);
    if (has_mode) {
        visit_type_str(v, &mode, "mode", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_file_close(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t handle = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &handle, "handle", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_guest_file_close(handle, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &handle, "handle", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_file_read(GuestFileRead *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestFileRead(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileRead(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_file_read(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestFileRead *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t handle = {0};
    bool has_count = false;
    int64_t count = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &handle, "handle", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_count, "count", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_count) {
        visit_type_int(v, &count, "count", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_guest_file_read(handle, has_count, count, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_file_read(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &handle, "handle", NULL);
    visit_optional(v, &has_count, "count", NULL);
    if (has_count) {
        visit_type_int(v, &count, "count", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_file_write(GuestFileWrite *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestFileWrite(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileWrite(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_file_write(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestFileWrite *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t handle = {0};
    char *buf_b64 = NULL;
    bool has_count = false;
    int64_t count = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &handle, "handle", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &buf_b64, "buf-b64", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_count, "count", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_count) {
        visit_type_int(v, &count, "count", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_guest_file_write(handle, buf_b64, has_count, count, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_file_write(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &handle, "handle", NULL);
    visit_type_str(v, &buf_b64, "buf-b64", NULL);
    visit_optional(v, &has_count, "count", NULL);
    if (has_count) {
        visit_type_int(v, &count, "count", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_file_seek(GuestFileSeek *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestFileSeek(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileSeek(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_file_seek(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestFileSeek *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t handle = {0};
    int64_t offset = {0};
    int64_t whence = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &handle, "handle", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &offset, "offset", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &whence, "whence", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_guest_file_seek(handle, offset, whence, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_file_seek(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &handle, "handle", NULL);
    visit_type_int(v, &offset, "offset", NULL);
    visit_type_int(v, &whence, "whence", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_file_flush(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t handle = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &handle, "handle", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_guest_file_flush(handle, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &handle, "handle", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_fsfreeze_status(GuestFsfreezeStatus ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestFsfreezeStatus(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFsfreezeStatus(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_fsfreeze_status(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestFsfreezeStatus retval;

    (void)args;
    retval = qmp_guest_fsfreeze_status(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_fsfreeze_status(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_fsfreeze_freeze(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_fsfreeze_freeze(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;

    (void)args;
    retval = qmp_guest_fsfreeze_freeze(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_fsfreeze_freeze(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_fsfreeze_freeze_list(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_fsfreeze_freeze_list(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_mountpoints = false;
    strList *mountpoints = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_mountpoints, "mountpoints", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_mountpoints) {
        visit_type_strList(v, &mountpoints, "mountpoints", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_guest_fsfreeze_freeze_list(has_mountpoints, mountpoints, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_fsfreeze_freeze_list(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_mountpoints, "mountpoints", NULL);
    if (has_mountpoints) {
        visit_type_strList(v, &mountpoints, "mountpoints", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_fsfreeze_thaw(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_fsfreeze_thaw(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;

    (void)args;
    retval = qmp_guest_fsfreeze_thaw(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_fsfreeze_thaw(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_input_guest_fstrim(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_minimum = false;
    int64_t minimum = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_minimum, "minimum", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_minimum) {
        visit_type_int(v, &minimum, "minimum", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_guest_fstrim(has_minimum, minimum, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_minimum, "minimum", NULL);
    if (has_minimum) {
        visit_type_int(v, &minimum, "minimum", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_suspend_disk(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_guest_suspend_disk(&local_err);
    error_propagate(errp, local_err);

}

static void qmp_marshal_input_guest_suspend_ram(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_guest_suspend_ram(&local_err);
    error_propagate(errp, local_err);

}

static void qmp_marshal_input_guest_suspend_hybrid(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_guest_suspend_hybrid(&local_err);
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_network_get_interfaces(GuestNetworkInterfaceList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestNetworkInterfaceList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestNetworkInterfaceList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_network_get_interfaces(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestNetworkInterfaceList *retval = NULL;

    (void)args;
    retval = qmp_guest_network_get_interfaces(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_network_get_interfaces(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_get_vcpus(GuestLogicalProcessorList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestLogicalProcessorList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestLogicalProcessorList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_get_vcpus(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestLogicalProcessorList *retval = NULL;

    (void)args;
    retval = qmp_guest_get_vcpus(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_get_vcpus(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_set_vcpus(int64_t ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_int(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_set_vcpus(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    GuestLogicalProcessorList *vcpus = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_GuestLogicalProcessorList(v, &vcpus, "vcpus", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_guest_set_vcpus(vcpus, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_set_vcpus(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestLogicalProcessorList(v, &vcpus, "vcpus", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_get_fsinfo(GuestFilesystemInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestFilesystemInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFilesystemInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_get_fsinfo(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestFilesystemInfoList *retval = NULL;

    (void)args;
    retval = qmp_guest_get_fsinfo(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_get_fsinfo(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_input_guest_set_user_password(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *username = NULL;
    char *password = NULL;
    bool crypted = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &username, "username", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &password, "password", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_bool(v, &crypted, "crypted", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_guest_set_user_password(username, password, crypted, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &username, "username", NULL);
    visit_type_str(v, &password, "password", NULL);
    visit_type_bool(v, &crypted, "crypted", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_get_memory_blocks(GuestMemoryBlockList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestMemoryBlockList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_get_memory_blocks(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestMemoryBlockList *retval = NULL;

    (void)args;
    retval = qmp_guest_get_memory_blocks(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_get_memory_blocks(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_guest_set_memory_blocks(GuestMemoryBlockResponseList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestMemoryBlockResponseList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockResponseList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_set_memory_blocks(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestMemoryBlockResponseList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    GuestMemoryBlockList *mem_blks = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_GuestMemoryBlockList(v, &mem_blks, "mem-blks", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_guest_set_memory_blocks(mem_blks, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_set_memory_blocks(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockList(v, &mem_blks, "mem-blks", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_guest_get_memory_block_info(GuestMemoryBlockInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_GuestMemoryBlockInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_input_guest_get_memory_block_info(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    GuestMemoryBlockInfo *retval = NULL;

    (void)args;
    retval = qmp_guest_get_memory_block_info(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_guest_get_memory_block_info(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_init_marshal(void)
{
    qmp_register_command("guest-sync-delimited", qmp_marshal_input_guest_sync_delimited, QCO_NO_OPTIONS);
    qmp_register_command("guest-sync", qmp_marshal_input_guest_sync, QCO_NO_OPTIONS);
    qmp_register_command("guest-ping", qmp_marshal_input_guest_ping, QCO_NO_OPTIONS);
    qmp_register_command("guest-get-time", qmp_marshal_input_guest_get_time, QCO_NO_OPTIONS);
    qmp_register_command("guest-set-time", qmp_marshal_input_guest_set_time, QCO_NO_OPTIONS);
    qmp_register_command("guest-info", qmp_marshal_input_guest_info, QCO_NO_OPTIONS);
    qmp_register_command("guest-shutdown", qmp_marshal_input_guest_shutdown, QCO_NO_SUCCESS_RESP);
    qmp_register_command("guest-file-open", qmp_marshal_input_guest_file_open, QCO_NO_OPTIONS);
    qmp_register_command("guest-file-close", qmp_marshal_input_guest_file_close, QCO_NO_OPTIONS);
    qmp_register_command("guest-file-read", qmp_marshal_input_guest_file_read, QCO_NO_OPTIONS);
    qmp_register_command("guest-file-write", qmp_marshal_input_guest_file_write, QCO_NO_OPTIONS);
    qmp_register_command("guest-file-seek", qmp_marshal_input_guest_file_seek, QCO_NO_OPTIONS);
    qmp_register_command("guest-file-flush", qmp_marshal_input_guest_file_flush, QCO_NO_OPTIONS);
    qmp_register_command("guest-fsfreeze-status", qmp_marshal_input_guest_fsfreeze_status, QCO_NO_OPTIONS);
    qmp_register_command("guest-fsfreeze-freeze", qmp_marshal_input_guest_fsfreeze_freeze, QCO_NO_OPTIONS);
    qmp_register_command("guest-fsfreeze-freeze-list", qmp_marshal_input_guest_fsfreeze_freeze_list, QCO_NO_OPTIONS);
    qmp_register_command("guest-fsfreeze-thaw", qmp_marshal_input_guest_fsfreeze_thaw, QCO_NO_OPTIONS);
    qmp_register_command("guest-fstrim", qmp_marshal_input_guest_fstrim, QCO_NO_OPTIONS);
    qmp_register_command("guest-suspend-disk", qmp_marshal_input_guest_suspend_disk, QCO_NO_SUCCESS_RESP);
    qmp_register_command("guest-suspend-ram", qmp_marshal_input_guest_suspend_ram, QCO_NO_SUCCESS_RESP);
    qmp_register_command("guest-suspend-hybrid", qmp_marshal_input_guest_suspend_hybrid, QCO_NO_SUCCESS_RESP);
    qmp_register_command("guest-network-get-interfaces", qmp_marshal_input_guest_network_get_interfaces, QCO_NO_OPTIONS);
    qmp_register_command("guest-get-vcpus", qmp_marshal_input_guest_get_vcpus, QCO_NO_OPTIONS);
    qmp_register_command("guest-set-vcpus", qmp_marshal_input_guest_set_vcpus, QCO_NO_OPTIONS);
    qmp_register_command("guest-get-fsinfo", qmp_marshal_input_guest_get_fsinfo, QCO_NO_OPTIONS);
    qmp_register_command("guest-set-user-password", qmp_marshal_input_guest_set_user_password, QCO_NO_OPTIONS);
    qmp_register_command("guest-get-memory-blocks", qmp_marshal_input_guest_get_memory_blocks, QCO_NO_OPTIONS);
    qmp_register_command("guest-set-memory-blocks", qmp_marshal_input_guest_set_memory_blocks, QCO_NO_OPTIONS);
    qmp_register_command("guest-get-memory-block-info", qmp_marshal_input_guest_get_memory_block_info, QCO_NO_OPTIONS);
}

qapi_init(qmp_init_marshal);
