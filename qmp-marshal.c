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
#include "qapi-types.h"
#include "qapi-visit.h"
#include "qmp-commands.h"

static void qmp_marshal_output_query_version(VersionInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_VersionInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VersionInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_version(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    VersionInfo *retval = NULL;

    (void)args;
    retval = qmp_query_version(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_version(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_commands(CommandInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_CommandInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_commands(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    CommandInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_commands(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_commands(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_block(BlockInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_BlockInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_block(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    BlockInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_block(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_block(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_blockstats(BlockStatsList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_BlockStatsList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockStatsList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_blockstats(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    BlockStatsList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_query_nodes = false;
    bool query_nodes = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_query_nodes, "query-nodes", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_query_nodes) {
        visit_type_bool(v, &query_nodes, "query-nodes", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_query_blockstats(has_query_nodes, query_nodes, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_blockstats(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_query_nodes, "query-nodes", NULL);
    if (has_query_nodes) {
        visit_type_bool(v, &query_nodes, "query-nodes", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_block_jobs(BlockJobInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_BlockJobInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockJobInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_block_jobs(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    BlockJobInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_block_jobs(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_block_jobs(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_block_passwd(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_device = false;
    char *device = NULL;
    bool has_node_name = false;
    char *node_name = NULL;
    char *password = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_device) {
        visit_type_str(v, &device, "device", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_node_name, "node-name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_type_str(v, &password, "password", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_passwd(has_device, device, has_node_name, node_name, password, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_device, "device", NULL);
    if (has_device) {
        visit_type_str(v, &device, "device", NULL);
    }
    visit_optional(v, &has_node_name, "node-name", NULL);
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", NULL);
    }
    visit_type_str(v, &password, "password", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_resize(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_device = false;
    char *device = NULL;
    bool has_node_name = false;
    char *node_name = NULL;
    int64_t size = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_device) {
        visit_type_str(v, &device, "device", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_node_name, "node-name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_type_int(v, &size, "size", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_resize(has_device, device, has_node_name, node_name, size, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_device, "device", NULL);
    if (has_device) {
        visit_type_str(v, &device, "device", NULL);
    }
    visit_optional(v, &has_node_name, "node-name", NULL);
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", NULL);
    }
    visit_type_int(v, &size, "size", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_blockdev_snapshot_sync(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_device = false;
    char *device = NULL;
    bool has_node_name = false;
    char *node_name = NULL;
    char *snapshot_file = NULL;
    bool has_snapshot_node_name = false;
    char *snapshot_node_name = NULL;
    bool has_format = false;
    char *format = NULL;
    bool has_mode = false;
    NewImageMode mode = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_device) {
        visit_type_str(v, &device, "device", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_node_name, "node-name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_type_str(v, &snapshot_file, "snapshot-file", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_snapshot_node_name, "snapshot-node-name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_snapshot_node_name) {
        visit_type_str(v, &snapshot_node_name, "snapshot-node-name", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_format, "format", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_format) {
        visit_type_str(v, &format, "format", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_mode, "mode", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_mode) {
        visit_type_NewImageMode(v, &mode, "mode", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_blockdev_snapshot_sync(has_device, device, has_node_name, node_name, snapshot_file, has_snapshot_node_name, snapshot_node_name, has_format, format, has_mode, mode, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_device, "device", NULL);
    if (has_device) {
        visit_type_str(v, &device, "device", NULL);
    }
    visit_optional(v, &has_node_name, "node-name", NULL);
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", NULL);
    }
    visit_type_str(v, &snapshot_file, "snapshot-file", NULL);
    visit_optional(v, &has_snapshot_node_name, "snapshot-node-name", NULL);
    if (has_snapshot_node_name) {
        visit_type_str(v, &snapshot_node_name, "snapshot-node-name", NULL);
    }
    visit_optional(v, &has_format, "format", NULL);
    if (has_format) {
        visit_type_str(v, &format, "format", NULL);
    }
    visit_optional(v, &has_mode, "mode", NULL);
    if (has_mode) {
        visit_type_NewImageMode(v, &mode, "mode", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_change_backing_file(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *image_node_name = NULL;
    char *backing_file = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &image_node_name, "image-node-name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &backing_file, "backing-file", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_change_backing_file(device, image_node_name, backing_file, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &image_node_name, "image-node-name", NULL);
    visit_type_str(v, &backing_file, "backing-file", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_commit(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    bool has_base = false;
    char *base = NULL;
    bool has_top = false;
    char *top = NULL;
    bool has_backing_file = false;
    char *backing_file = NULL;
    bool has_speed = false;
    int64_t speed = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_base, "base", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_base) {
        visit_type_str(v, &base, "base", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_top, "top", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_top) {
        visit_type_str(v, &top, "top", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_backing_file, "backing-file", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_backing_file) {
        visit_type_str(v, &backing_file, "backing-file", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_speed, "speed", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_speed) {
        visit_type_int(v, &speed, "speed", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_block_commit(device, has_base, base, has_top, top, has_backing_file, backing_file, has_speed, speed, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_optional(v, &has_base, "base", NULL);
    if (has_base) {
        visit_type_str(v, &base, "base", NULL);
    }
    visit_optional(v, &has_top, "top", NULL);
    if (has_top) {
        visit_type_str(v, &top, "top", NULL);
    }
    visit_optional(v, &has_backing_file, "backing-file", NULL);
    if (has_backing_file) {
        visit_type_str(v, &backing_file, "backing-file", NULL);
    }
    visit_optional(v, &has_speed, "speed", NULL);
    if (has_speed) {
        visit_type_int(v, &speed, "speed", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_drive_backup(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *target = NULL;
    bool has_format = false;
    char *format = NULL;
    MirrorSyncMode sync = {0};
    bool has_mode = false;
    NewImageMode mode = {0};
    bool has_speed = false;
    int64_t speed = {0};
    bool has_bitmap = false;
    char *bitmap = NULL;
    bool has_on_source_error = false;
    BlockdevOnError on_source_error = {0};
    bool has_on_target_error = false;
    BlockdevOnError on_target_error = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &target, "target", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_format, "format", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_format) {
        visit_type_str(v, &format, "format", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_type_MirrorSyncMode(v, &sync, "sync", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_mode, "mode", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_mode) {
        visit_type_NewImageMode(v, &mode, "mode", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_speed, "speed", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_speed) {
        visit_type_int(v, &speed, "speed", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_bitmap, "bitmap", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_bitmap) {
        visit_type_str(v, &bitmap, "bitmap", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_source_error, "on-source-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_source_error) {
        visit_type_BlockdevOnError(v, &on_source_error, "on-source-error", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_target_error, "on-target-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_target_error) {
        visit_type_BlockdevOnError(v, &on_target_error, "on-target-error", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_drive_backup(device, target, has_format, format, sync, has_mode, mode, has_speed, speed, has_bitmap, bitmap, has_on_source_error, on_source_error, has_on_target_error, on_target_error, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &target, "target", NULL);
    visit_optional(v, &has_format, "format", NULL);
    if (has_format) {
        visit_type_str(v, &format, "format", NULL);
    }
    visit_type_MirrorSyncMode(v, &sync, "sync", NULL);
    visit_optional(v, &has_mode, "mode", NULL);
    if (has_mode) {
        visit_type_NewImageMode(v, &mode, "mode", NULL);
    }
    visit_optional(v, &has_speed, "speed", NULL);
    if (has_speed) {
        visit_type_int(v, &speed, "speed", NULL);
    }
    visit_optional(v, &has_bitmap, "bitmap", NULL);
    if (has_bitmap) {
        visit_type_str(v, &bitmap, "bitmap", NULL);
    }
    visit_optional(v, &has_on_source_error, "on-source-error", NULL);
    if (has_on_source_error) {
        visit_type_BlockdevOnError(v, &on_source_error, "on-source-error", NULL);
    }
    visit_optional(v, &has_on_target_error, "on-target-error", NULL);
    if (has_on_target_error) {
        visit_type_BlockdevOnError(v, &on_target_error, "on-target-error", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_blockdev_backup(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *target = NULL;
    MirrorSyncMode sync = {0};
    bool has_speed = false;
    int64_t speed = {0};
    bool has_on_source_error = false;
    BlockdevOnError on_source_error = {0};
    bool has_on_target_error = false;
    BlockdevOnError on_target_error = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &target, "target", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_MirrorSyncMode(v, &sync, "sync", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_speed, "speed", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_speed) {
        visit_type_int(v, &speed, "speed", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_source_error, "on-source-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_source_error) {
        visit_type_BlockdevOnError(v, &on_source_error, "on-source-error", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_target_error, "on-target-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_target_error) {
        visit_type_BlockdevOnError(v, &on_target_error, "on-target-error", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_blockdev_backup(device, target, sync, has_speed, speed, has_on_source_error, on_source_error, has_on_target_error, on_target_error, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &target, "target", NULL);
    visit_type_MirrorSyncMode(v, &sync, "sync", NULL);
    visit_optional(v, &has_speed, "speed", NULL);
    if (has_speed) {
        visit_type_int(v, &speed, "speed", NULL);
    }
    visit_optional(v, &has_on_source_error, "on-source-error", NULL);
    if (has_on_source_error) {
        visit_type_BlockdevOnError(v, &on_source_error, "on-source-error", NULL);
    }
    visit_optional(v, &has_on_target_error, "on-target-error", NULL);
    if (has_on_target_error) {
        visit_type_BlockdevOnError(v, &on_target_error, "on-target-error", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_named_block_nodes(BlockDeviceInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_BlockDeviceInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockDeviceInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_named_block_nodes(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    BlockDeviceInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_named_block_nodes(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_named_block_nodes(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_drive_mirror(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *target = NULL;
    bool has_format = false;
    char *format = NULL;
    bool has_node_name = false;
    char *node_name = NULL;
    bool has_replaces = false;
    char *replaces = NULL;
    MirrorSyncMode sync = {0};
    bool has_mode = false;
    NewImageMode mode = {0};
    bool has_speed = false;
    int64_t speed = {0};
    bool has_granularity = false;
    uint32_t granularity = {0};
    bool has_buf_size = false;
    int64_t buf_size = {0};
    bool has_on_source_error = false;
    BlockdevOnError on_source_error = {0};
    bool has_on_target_error = false;
    BlockdevOnError on_target_error = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &target, "target", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_format, "format", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_format) {
        visit_type_str(v, &format, "format", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_node_name, "node-name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_replaces, "replaces", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_replaces) {
        visit_type_str(v, &replaces, "replaces", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_type_MirrorSyncMode(v, &sync, "sync", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_mode, "mode", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_mode) {
        visit_type_NewImageMode(v, &mode, "mode", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_speed, "speed", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_speed) {
        visit_type_int(v, &speed, "speed", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_granularity, "granularity", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_granularity) {
        visit_type_uint32(v, &granularity, "granularity", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_buf_size, "buf-size", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_buf_size) {
        visit_type_int(v, &buf_size, "buf-size", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_source_error, "on-source-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_source_error) {
        visit_type_BlockdevOnError(v, &on_source_error, "on-source-error", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_target_error, "on-target-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_target_error) {
        visit_type_BlockdevOnError(v, &on_target_error, "on-target-error", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_drive_mirror(device, target, has_format, format, has_node_name, node_name, has_replaces, replaces, sync, has_mode, mode, has_speed, speed, has_granularity, granularity, has_buf_size, buf_size, has_on_source_error, on_source_error, has_on_target_error, on_target_error, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &target, "target", NULL);
    visit_optional(v, &has_format, "format", NULL);
    if (has_format) {
        visit_type_str(v, &format, "format", NULL);
    }
    visit_optional(v, &has_node_name, "node-name", NULL);
    if (has_node_name) {
        visit_type_str(v, &node_name, "node-name", NULL);
    }
    visit_optional(v, &has_replaces, "replaces", NULL);
    if (has_replaces) {
        visit_type_str(v, &replaces, "replaces", NULL);
    }
    visit_type_MirrorSyncMode(v, &sync, "sync", NULL);
    visit_optional(v, &has_mode, "mode", NULL);
    if (has_mode) {
        visit_type_NewImageMode(v, &mode, "mode", NULL);
    }
    visit_optional(v, &has_speed, "speed", NULL);
    if (has_speed) {
        visit_type_int(v, &speed, "speed", NULL);
    }
    visit_optional(v, &has_granularity, "granularity", NULL);
    if (has_granularity) {
        visit_type_uint32(v, &granularity, "granularity", NULL);
    }
    visit_optional(v, &has_buf_size, "buf-size", NULL);
    if (has_buf_size) {
        visit_type_int(v, &buf_size, "buf-size", NULL);
    }
    visit_optional(v, &has_on_source_error, "on-source-error", NULL);
    if (has_on_source_error) {
        visit_type_BlockdevOnError(v, &on_source_error, "on-source-error", NULL);
    }
    visit_optional(v, &has_on_target_error, "on-target-error", NULL);
    if (has_on_target_error) {
        visit_type_BlockdevOnError(v, &on_target_error, "on-target-error", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_dirty_bitmap_add(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *node = NULL;
    char *name = NULL;
    bool has_granularity = false;
    uint32_t granularity = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &node, "node", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_granularity, "granularity", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_granularity) {
        visit_type_uint32(v, &granularity, "granularity", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_block_dirty_bitmap_add(node, name, has_granularity, granularity, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &node, "node", NULL);
    visit_type_str(v, &name, "name", NULL);
    visit_optional(v, &has_granularity, "granularity", NULL);
    if (has_granularity) {
        visit_type_uint32(v, &granularity, "granularity", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_dirty_bitmap_remove(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *node = NULL;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &node, "node", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_dirty_bitmap_remove(node, name, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &node, "node", NULL);
    visit_type_str(v, &name, "name", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_dirty_bitmap_clear(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *node = NULL;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &node, "node", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_dirty_bitmap_clear(node, name, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &node, "node", NULL);
    visit_type_str(v, &name, "name", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_set_io_throttle(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    int64_t bps = {0};
    int64_t bps_rd = {0};
    int64_t bps_wr = {0};
    int64_t iops = {0};
    int64_t iops_rd = {0};
    int64_t iops_wr = {0};
    bool has_bps_max = false;
    int64_t bps_max = {0};
    bool has_bps_rd_max = false;
    int64_t bps_rd_max = {0};
    bool has_bps_wr_max = false;
    int64_t bps_wr_max = {0};
    bool has_iops_max = false;
    int64_t iops_max = {0};
    bool has_iops_rd_max = false;
    int64_t iops_rd_max = {0};
    bool has_iops_wr_max = false;
    int64_t iops_wr_max = {0};
    bool has_iops_size = false;
    int64_t iops_size = {0};
    bool has_group = false;
    char *group = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &bps, "bps", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &bps_rd, "bps_rd", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &bps_wr, "bps_wr", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &iops, "iops", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &iops_rd, "iops_rd", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &iops_wr, "iops_wr", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_bps_max, "bps_max", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_bps_max) {
        visit_type_int(v, &bps_max, "bps_max", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_bps_rd_max, "bps_rd_max", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_bps_rd_max) {
        visit_type_int(v, &bps_rd_max, "bps_rd_max", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_bps_wr_max, "bps_wr_max", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_bps_wr_max) {
        visit_type_int(v, &bps_wr_max, "bps_wr_max", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_iops_max, "iops_max", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_iops_max) {
        visit_type_int(v, &iops_max, "iops_max", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_iops_rd_max, "iops_rd_max", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_iops_rd_max) {
        visit_type_int(v, &iops_rd_max, "iops_rd_max", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_iops_wr_max, "iops_wr_max", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_iops_wr_max) {
        visit_type_int(v, &iops_wr_max, "iops_wr_max", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_iops_size, "iops_size", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_iops_size) {
        visit_type_int(v, &iops_size, "iops_size", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_group, "group", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_group) {
        visit_type_str(v, &group, "group", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_block_set_io_throttle(device, bps, bps_rd, bps_wr, iops, iops_rd, iops_wr, has_bps_max, bps_max, has_bps_rd_max, bps_rd_max, has_bps_wr_max, bps_wr_max, has_iops_max, iops_max, has_iops_rd_max, iops_rd_max, has_iops_wr_max, iops_wr_max, has_iops_size, iops_size, has_group, group, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_int(v, &bps, "bps", NULL);
    visit_type_int(v, &bps_rd, "bps_rd", NULL);
    visit_type_int(v, &bps_wr, "bps_wr", NULL);
    visit_type_int(v, &iops, "iops", NULL);
    visit_type_int(v, &iops_rd, "iops_rd", NULL);
    visit_type_int(v, &iops_wr, "iops_wr", NULL);
    visit_optional(v, &has_bps_max, "bps_max", NULL);
    if (has_bps_max) {
        visit_type_int(v, &bps_max, "bps_max", NULL);
    }
    visit_optional(v, &has_bps_rd_max, "bps_rd_max", NULL);
    if (has_bps_rd_max) {
        visit_type_int(v, &bps_rd_max, "bps_rd_max", NULL);
    }
    visit_optional(v, &has_bps_wr_max, "bps_wr_max", NULL);
    if (has_bps_wr_max) {
        visit_type_int(v, &bps_wr_max, "bps_wr_max", NULL);
    }
    visit_optional(v, &has_iops_max, "iops_max", NULL);
    if (has_iops_max) {
        visit_type_int(v, &iops_max, "iops_max", NULL);
    }
    visit_optional(v, &has_iops_rd_max, "iops_rd_max", NULL);
    if (has_iops_rd_max) {
        visit_type_int(v, &iops_rd_max, "iops_rd_max", NULL);
    }
    visit_optional(v, &has_iops_wr_max, "iops_wr_max", NULL);
    if (has_iops_wr_max) {
        visit_type_int(v, &iops_wr_max, "iops_wr_max", NULL);
    }
    visit_optional(v, &has_iops_size, "iops_size", NULL);
    if (has_iops_size) {
        visit_type_int(v, &iops_size, "iops_size", NULL);
    }
    visit_optional(v, &has_group, "group", NULL);
    if (has_group) {
        visit_type_str(v, &group, "group", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_stream(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    bool has_base = false;
    char *base = NULL;
    bool has_backing_file = false;
    char *backing_file = NULL;
    bool has_speed = false;
    int64_t speed = {0};
    bool has_on_error = false;
    BlockdevOnError on_error = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_base, "base", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_base) {
        visit_type_str(v, &base, "base", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_backing_file, "backing-file", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_backing_file) {
        visit_type_str(v, &backing_file, "backing-file", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_speed, "speed", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_speed) {
        visit_type_int(v, &speed, "speed", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_on_error, "on-error", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_on_error) {
        visit_type_BlockdevOnError(v, &on_error, "on-error", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_block_stream(device, has_base, base, has_backing_file, backing_file, has_speed, speed, has_on_error, on_error, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_optional(v, &has_base, "base", NULL);
    if (has_base) {
        visit_type_str(v, &base, "base", NULL);
    }
    visit_optional(v, &has_backing_file, "backing-file", NULL);
    if (has_backing_file) {
        visit_type_str(v, &backing_file, "backing-file", NULL);
    }
    visit_optional(v, &has_speed, "speed", NULL);
    if (has_speed) {
        visit_type_int(v, &speed, "speed", NULL);
    }
    visit_optional(v, &has_on_error, "on-error", NULL);
    if (has_on_error) {
        visit_type_BlockdevOnError(v, &on_error, "on-error", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_job_set_speed(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    int64_t speed = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &speed, "speed", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_job_set_speed(device, speed, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_int(v, &speed, "speed", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_job_cancel(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    bool has_force = false;
    bool force = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_force, "force", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_force) {
        visit_type_bool(v, &force, "force", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_block_job_cancel(device, has_force, force, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_optional(v, &has_force, "force", NULL);
    if (has_force) {
        visit_type_bool(v, &force, "force", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_job_pause(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_job_pause(device, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_job_resume(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_job_resume(device, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_job_complete(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_job_complete(device, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_blockdev_add(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    BlockdevOptions *options = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_BlockdevOptions(v, &options, "options", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_blockdev_add(options, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BlockdevOptions(v, &options, "options", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_block_set_write_threshold(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *node_name = NULL;
    uint64_t write_threshold = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &node_name, "node-name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_uint64(v, &write_threshold, "write-threshold", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_block_set_write_threshold(node_name, write_threshold, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &node_name, "node-name", NULL);
    visit_type_uint64(v, &write_threshold, "write-threshold", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_blockdev_snapshot_internal_sync(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_blockdev_snapshot_internal_sync(device, name, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &name, "name", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_blockdev_snapshot_delete_internal_sync(SnapshotInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_SnapshotInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SnapshotInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_blockdev_snapshot_delete_internal_sync(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    SnapshotInfo *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    bool has_id = false;
    char *id = NULL;
    bool has_name = false;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_id, "id", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_id) {
        visit_type_str(v, &id, "id", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_name) {
        visit_type_str(v, &name, "name", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_blockdev_snapshot_delete_internal_sync(device, has_id, id, has_name, name, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_blockdev_snapshot_delete_internal_sync(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_optional(v, &has_id, "id", NULL);
    if (has_id) {
        visit_type_str(v, &id, "id", NULL);
    }
    visit_optional(v, &has_name, "name", NULL);
    if (has_name) {
        visit_type_str(v, &name, "name", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_eject(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    bool has_force = false;
    bool force = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_force, "force", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_force) {
        visit_type_bool(v, &force, "force", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_eject(device, has_force, force, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_optional(v, &has_force, "force", NULL);
    if (has_force) {
        visit_type_bool(v, &force, "force", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_nbd_server_start(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    SocketAddress *addr = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_SocketAddress(v, &addr, "addr", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_nbd_server_start(addr, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SocketAddress(v, &addr, "addr", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_nbd_server_add(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    bool has_writable = false;
    bool writable = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_writable, "writable", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_writable) {
        visit_type_bool(v, &writable, "writable", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_nbd_server_add(device, has_writable, writable, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_optional(v, &has_writable, "writable", NULL);
    if (has_writable) {
        visit_type_bool(v, &writable, "writable", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_nbd_server_stop(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_nbd_server_stop(&local_err);
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_trace_event_get_state(TraceEventInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_TraceEventInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TraceEventInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_trace_event_get_state(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    TraceEventInfoList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_trace_event_get_state(name, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_trace_event_get_state(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_trace_event_set_state(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;
    bool enable = {0};
    bool has_ignore_unavailable = false;
    bool ignore_unavailable = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_bool(v, &enable, "enable", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_ignore_unavailable, "ignore-unavailable", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_ignore_unavailable) {
        visit_type_bool(v, &ignore_unavailable, "ignore-unavailable", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_trace_event_set_state(name, enable, has_ignore_unavailable, ignore_unavailable, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    visit_type_bool(v, &enable, "enable", NULL);
    visit_optional(v, &has_ignore_unavailable, "ignore-unavailable", NULL);
    if (has_ignore_unavailable) {
        visit_type_bool(v, &ignore_unavailable, "ignore-unavailable", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_add_client(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *protocol = NULL;
    char *fdname = NULL;
    bool has_skipauth = false;
    bool skipauth = {0};
    bool has_tls = false;
    bool tls = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &protocol, "protocol", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &fdname, "fdname", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_skipauth, "skipauth", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_skipauth) {
        visit_type_bool(v, &skipauth, "skipauth", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_tls, "tls", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_tls) {
        visit_type_bool(v, &tls, "tls", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_add_client(protocol, fdname, has_skipauth, skipauth, has_tls, tls, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &protocol, "protocol", NULL);
    visit_type_str(v, &fdname, "fdname", NULL);
    visit_optional(v, &has_skipauth, "skipauth", NULL);
    if (has_skipauth) {
        visit_type_bool(v, &skipauth, "skipauth", NULL);
    }
    visit_optional(v, &has_tls, "tls", NULL);
    if (has_tls) {
        visit_type_bool(v, &tls, "tls", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_name(NameInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_NameInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_NameInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_name(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    NameInfo *retval = NULL;

    (void)args;
    retval = qmp_query_name(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_name(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_kvm(KvmInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_KvmInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_KvmInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_kvm(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    KvmInfo *retval = NULL;

    (void)args;
    retval = qmp_query_kvm(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_kvm(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_status(StatusInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_StatusInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_StatusInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_status(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    StatusInfo *retval = NULL;

    (void)args;
    retval = qmp_query_status(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_status(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_uuid(UuidInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_UuidInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_UuidInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_uuid(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    UuidInfo *retval = NULL;

    (void)args;
    retval = qmp_query_uuid(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_uuid(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_chardev(ChardevInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_ChardevInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_chardev(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    ChardevInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_chardev(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_chardev(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_chardev_backends(ChardevBackendInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_ChardevBackendInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevBackendInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_chardev_backends(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    ChardevBackendInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_chardev_backends(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_chardev_backends(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_ringbuf_write(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *data = NULL;
    bool has_format = false;
    DataFormat format = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &data, "data", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_format, "format", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_format) {
        visit_type_DataFormat(v, &format, "format", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_ringbuf_write(device, data, has_format, format, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &data, "data", NULL);
    visit_optional(v, &has_format, "format", NULL);
    if (has_format) {
        visit_type_DataFormat(v, &format, "format", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_ringbuf_read(char *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_str(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_ringbuf_read(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    char *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    int64_t size = {0};
    bool has_format = false;
    DataFormat format = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &size, "size", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_format, "format", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_format) {
        visit_type_DataFormat(v, &format, "format", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_ringbuf_read(device, size, has_format, format, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_ringbuf_read(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_int(v, &size, "size", NULL);
    visit_optional(v, &has_format, "format", NULL);
    if (has_format) {
        visit_type_DataFormat(v, &format, "format", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_events(EventInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_EventInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_EventInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_events(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    EventInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_events(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_events(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_migrate(MigrationInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MigrationInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_migrate(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MigrationInfo *retval = NULL;

    (void)args;
    retval = qmp_query_migrate(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_migrate(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_migrate_set_capabilities(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    MigrationCapabilityStatusList *capabilities = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_MigrationCapabilityStatusList(v, &capabilities, "capabilities", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_migrate_set_capabilities(capabilities, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationCapabilityStatusList(v, &capabilities, "capabilities", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_migrate_capabilities(MigrationCapabilityStatusList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MigrationCapabilityStatusList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationCapabilityStatusList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_migrate_capabilities(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MigrationCapabilityStatusList *retval = NULL;

    (void)args;
    retval = qmp_query_migrate_capabilities(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_migrate_capabilities(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_migrate_set_parameters(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_compress_level = false;
    int64_t compress_level = {0};
    bool has_compress_threads = false;
    int64_t compress_threads = {0};
    bool has_decompress_threads = false;
    int64_t decompress_threads = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_compress_level, "compress-level", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_compress_level) {
        visit_type_int(v, &compress_level, "compress-level", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_compress_threads, "compress-threads", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_compress_threads) {
        visit_type_int(v, &compress_threads, "compress-threads", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_decompress_threads, "decompress-threads", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_decompress_threads) {
        visit_type_int(v, &decompress_threads, "decompress-threads", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_migrate_set_parameters(has_compress_level, compress_level, has_compress_threads, compress_threads, has_decompress_threads, decompress_threads, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_compress_level, "compress-level", NULL);
    if (has_compress_level) {
        visit_type_int(v, &compress_level, "compress-level", NULL);
    }
    visit_optional(v, &has_compress_threads, "compress-threads", NULL);
    if (has_compress_threads) {
        visit_type_int(v, &compress_threads, "compress-threads", NULL);
    }
    visit_optional(v, &has_decompress_threads, "decompress-threads", NULL);
    if (has_decompress_threads) {
        visit_type_int(v, &decompress_threads, "decompress-threads", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_migrate_parameters(MigrationParameters *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MigrationParameters(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MigrationParameters(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_migrate_parameters(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MigrationParameters *retval = NULL;

    (void)args;
    retval = qmp_query_migrate_parameters(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_migrate_parameters(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_client_migrate_info(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *protocol = NULL;
    char *hostname = NULL;
    bool has_port = false;
    int64_t port = {0};
    bool has_tls_port = false;
    int64_t tls_port = {0};
    bool has_cert_subject = false;
    char *cert_subject = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &protocol, "protocol", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &hostname, "hostname", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_port, "port", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_port) {
        visit_type_int(v, &port, "port", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_tls_port, "tls-port", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_tls_port) {
        visit_type_int(v, &tls_port, "tls-port", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_cert_subject, "cert-subject", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_cert_subject) {
        visit_type_str(v, &cert_subject, "cert-subject", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_client_migrate_info(protocol, hostname, has_port, port, has_tls_port, tls_port, has_cert_subject, cert_subject, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &protocol, "protocol", NULL);
    visit_type_str(v, &hostname, "hostname", NULL);
    visit_optional(v, &has_port, "port", NULL);
    if (has_port) {
        visit_type_int(v, &port, "port", NULL);
    }
    visit_optional(v, &has_tls_port, "tls-port", NULL);
    if (has_tls_port) {
        visit_type_int(v, &tls_port, "tls-port", NULL);
    }
    visit_optional(v, &has_cert_subject, "cert-subject", NULL);
    if (has_cert_subject) {
        visit_type_str(v, &cert_subject, "cert-subject", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_mice(MouseInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MouseInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MouseInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_mice(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MouseInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_mice(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_mice(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_cpus(CpuInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_CpuInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CpuInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_cpus(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    CpuInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_cpus(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_cpus(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_iothreads(IOThreadInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_IOThreadInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_IOThreadInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_iothreads(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    IOThreadInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_iothreads(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_iothreads(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_vnc(VncInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_VncInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_vnc(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    VncInfo *retval = NULL;

    (void)args;
    retval = qmp_query_vnc(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_vnc(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_vnc_servers(VncInfo2List *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_VncInfo2List(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_VncInfo2List(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_vnc_servers(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    VncInfo2List *retval = NULL;

    (void)args;
    retval = qmp_query_vnc_servers(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_vnc_servers(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_spice(SpiceInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_SpiceInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_SpiceInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_spice(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    SpiceInfo *retval = NULL;

    (void)args;
    retval = qmp_query_spice(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_spice(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_balloon(BalloonInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_BalloonInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_BalloonInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_balloon(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    BalloonInfo *retval = NULL;

    (void)args;
    retval = qmp_query_balloon(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_balloon(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_pci(PciInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_PciInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_PciInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_pci(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    PciInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_pci(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_pci(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_quit(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_quit(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_stop(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_stop(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_system_reset(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_system_reset(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_system_powerdown(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_system_powerdown(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_cpu(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t index = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &index, "index", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_cpu(index, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &index, "index", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_cpu_add(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t id = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_cpu_add(id, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_memsave(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t val = {0};
    int64_t size = {0};
    char *filename = NULL;
    bool has_cpu_index = false;
    int64_t cpu_index = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &val, "val", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &size, "size", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &filename, "filename", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_cpu_index, "cpu-index", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_cpu_index) {
        visit_type_int(v, &cpu_index, "cpu-index", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_memsave(val, size, filename, has_cpu_index, cpu_index, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &val, "val", NULL);
    visit_type_int(v, &size, "size", NULL);
    visit_type_str(v, &filename, "filename", NULL);
    visit_optional(v, &has_cpu_index, "cpu-index", NULL);
    if (has_cpu_index) {
        visit_type_int(v, &cpu_index, "cpu-index", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_pmemsave(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t val = {0};
    int64_t size = {0};
    char *filename = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &val, "val", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_int(v, &size, "size", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &filename, "filename", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_pmemsave(val, size, filename, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &val, "val", NULL);
    visit_type_int(v, &size, "size", NULL);
    visit_type_str(v, &filename, "filename", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_cont(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_cont(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_system_wakeup(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_system_wakeup(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_inject_nmi(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_inject_nmi(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_set_link(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;
    bool up = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_bool(v, &up, "up", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_set_link(name, up, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    visit_type_bool(v, &up, "up", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_balloon(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t value = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &value, "value", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_balloon(value, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &value, "value", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_transaction(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    TransactionActionList *actions = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_TransactionActionList(v, &actions, "actions", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_transaction(actions, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TransactionActionList(v, &actions, "actions", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_human_monitor_command(char *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_str(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_human_monitor_command(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    char *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *command_line = NULL;
    bool has_cpu_index = false;
    int64_t cpu_index = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &command_line, "command-line", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_cpu_index, "cpu-index", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_cpu_index) {
        visit_type_int(v, &cpu_index, "cpu-index", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_human_monitor_command(command_line, has_cpu_index, cpu_index, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_human_monitor_command(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &command_line, "command-line", NULL);
    visit_optional(v, &has_cpu_index, "cpu-index", NULL);
    if (has_cpu_index) {
        visit_type_int(v, &cpu_index, "cpu-index", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_migrate_cancel(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_migrate_cancel(&local_err);
    error_propagate(errp, local_err);

}

void qmp_marshal_input_migrate_set_downtime(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    double value = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_number(v, &value, "value", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_migrate_set_downtime(value, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_number(v, &value, "value", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_migrate_set_speed(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t value = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &value, "value", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_migrate_set_speed(value, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &value, "value", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_migrate_set_cache_size(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t value = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &value, "value", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_migrate_set_cache_size(value, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &value, "value", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_migrate_cache_size(int64_t ret_in, QObject **ret_out, Error **errp)
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

void qmp_marshal_input_query_migrate_cache_size(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    int64_t retval;

    (void)args;
    retval = qmp_query_migrate_cache_size(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_migrate_cache_size(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_qom_list(ObjectPropertyInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_ObjectPropertyInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ObjectPropertyInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_qom_list(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    ObjectPropertyInfoList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *path = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &path, "path", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_qom_list(path, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_qom_list(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &path, "path", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_set_password(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *protocol = NULL;
    char *password = NULL;
    bool has_connected = false;
    char *connected = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &protocol, "protocol", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &password, "password", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_connected, "connected", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_connected) {
        visit_type_str(v, &connected, "connected", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_set_password(protocol, password, has_connected, connected, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &protocol, "protocol", NULL);
    visit_type_str(v, &password, "password", NULL);
    visit_optional(v, &has_connected, "connected", NULL);
    if (has_connected) {
        visit_type_str(v, &connected, "connected", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_expire_password(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *protocol = NULL;
    char *time = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &protocol, "protocol", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &time, "time", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_expire_password(protocol, time, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &protocol, "protocol", NULL);
    visit_type_str(v, &time, "time", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_change_vnc_password(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *password = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &password, "password", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_change_vnc_password(password, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &password, "password", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_change(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *device = NULL;
    char *target = NULL;
    bool has_arg = false;
    char *arg = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &device, "device", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &target, "target", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_arg, "arg", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_arg) {
        visit_type_str(v, &arg, "arg", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_change(device, target, has_arg, arg, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &device, "device", NULL);
    visit_type_str(v, &target, "target", NULL);
    visit_optional(v, &has_arg, "arg", NULL);
    if (has_arg) {
        visit_type_str(v, &arg, "arg", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_qom_list_types(ObjectTypeInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_ObjectTypeInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ObjectTypeInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_qom_list_types(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    ObjectTypeInfoList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_implements = false;
    char *implements = NULL;
    bool has_abstract = false;
    bool abstract = {0};

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_implements, "implements", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_implements) {
        visit_type_str(v, &implements, "implements", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_abstract, "abstract", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_abstract) {
        visit_type_bool(v, &abstract, "abstract", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_qom_list_types(has_implements, implements, has_abstract, abstract, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_qom_list_types(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_implements, "implements", NULL);
    if (has_implements) {
        visit_type_str(v, &implements, "implements", NULL);
    }
    visit_optional(v, &has_abstract, "abstract", NULL);
    if (has_abstract) {
        visit_type_bool(v, &abstract, "abstract", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_device_list_properties(DevicePropertyInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_DevicePropertyInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DevicePropertyInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_device_list_properties(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    DevicePropertyInfoList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *q_typename = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &q_typename, "typename", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_device_list_properties(q_typename, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_device_list_properties(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &q_typename, "typename", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_migrate(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *uri = NULL;
    bool has_blk = false;
    bool blk = {0};
    bool has_inc = false;
    bool inc = {0};
    bool has_detach = false;
    bool detach = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &uri, "uri", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_blk, "blk", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_blk) {
        visit_type_bool(v, &blk, "blk", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_inc, "inc", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_inc) {
        visit_type_bool(v, &inc, "inc", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_detach, "detach", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_detach) {
        visit_type_bool(v, &detach, "detach", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_migrate(uri, has_blk, blk, has_inc, inc, has_detach, detach, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &uri, "uri", NULL);
    visit_optional(v, &has_blk, "blk", NULL);
    if (has_blk) {
        visit_type_bool(v, &blk, "blk", NULL);
    }
    visit_optional(v, &has_inc, "inc", NULL);
    if (has_inc) {
        visit_type_bool(v, &inc, "inc", NULL);
    }
    visit_optional(v, &has_detach, "detach", NULL);
    if (has_detach) {
        visit_type_bool(v, &detach, "detach", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_migrate_incoming(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *uri = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &uri, "uri", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_migrate_incoming(uri, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &uri, "uri", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_xen_save_devices_state(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *filename = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &filename, "filename", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_xen_save_devices_state(filename, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &filename, "filename", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_xen_set_global_dirty_log(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool enable = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_bool(v, &enable, "enable", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_xen_set_global_dirty_log(enable, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_bool(v, &enable, "enable", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_device_del(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *id = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_device_del(id, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_dump_guest_memory(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool paging = {0};
    char *protocol = NULL;
    bool has_begin = false;
    int64_t begin = {0};
    bool has_length = false;
    int64_t length = {0};
    bool has_format = false;
    DumpGuestMemoryFormat format = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_bool(v, &paging, "paging", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_str(v, &protocol, "protocol", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_begin, "begin", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_begin) {
        visit_type_int(v, &begin, "begin", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_length, "length", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_length) {
        visit_type_int(v, &length, "length", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_format, "format", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_format) {
        visit_type_DumpGuestMemoryFormat(v, &format, "format", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_dump_guest_memory(paging, protocol, has_begin, begin, has_length, length, has_format, format, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_bool(v, &paging, "paging", NULL);
    visit_type_str(v, &protocol, "protocol", NULL);
    visit_optional(v, &has_begin, "begin", NULL);
    if (has_begin) {
        visit_type_int(v, &begin, "begin", NULL);
    }
    visit_optional(v, &has_length, "length", NULL);
    if (has_length) {
        visit_type_int(v, &length, "length", NULL);
    }
    visit_optional(v, &has_format, "format", NULL);
    if (has_format) {
        visit_type_DumpGuestMemoryFormat(v, &format, "format", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_dump_guest_memory_capability(DumpGuestMemoryCapability *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_DumpGuestMemoryCapability(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_DumpGuestMemoryCapability(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_dump_guest_memory_capability(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    DumpGuestMemoryCapability *retval = NULL;

    (void)args;
    retval = qmp_query_dump_guest_memory_capability(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_dump_guest_memory_capability(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_netdev_del(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *id = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_netdev_del(id, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_object_del(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *id = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_object_del(id, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_getfd(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *fdname = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &fdname, "fdname", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_getfd(fdname, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &fdname, "fdname", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_closefd(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *fdname = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &fdname, "fdname", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_closefd(fdname, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &fdname, "fdname", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_machines(MachineInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MachineInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MachineInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_machines(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MachineInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_machines(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_machines(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_cpu_definitions(CpuDefinitionInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_CpuDefinitionInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CpuDefinitionInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_cpu_definitions(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    CpuDefinitionInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_cpu_definitions(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_cpu_definitions(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_add_fd(AddfdInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_AddfdInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_AddfdInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_add_fd(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    AddfdInfo *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_fdset_id = false;
    int64_t fdset_id = {0};
    bool has_opaque = false;
    char *opaque = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_fdset_id, "fdset-id", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_fdset_id) {
        visit_type_int(v, &fdset_id, "fdset-id", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_optional(v, &has_opaque, "opaque", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_opaque) {
        visit_type_str(v, &opaque, "opaque", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_add_fd(has_fdset_id, fdset_id, has_opaque, opaque, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_add_fd(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_fdset_id, "fdset-id", NULL);
    if (has_fdset_id) {
        visit_type_int(v, &fdset_id, "fdset-id", NULL);
    }
    visit_optional(v, &has_opaque, "opaque", NULL);
    if (has_opaque) {
        visit_type_str(v, &opaque, "opaque", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_remove_fd(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    int64_t fdset_id = {0};
    bool has_fd = false;
    int64_t fd = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_int(v, &fdset_id, "fdset-id", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_fd, "fd", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_fd) {
        visit_type_int(v, &fd, "fd", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_remove_fd(fdset_id, has_fd, fd, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_int(v, &fdset_id, "fdset-id", NULL);
    visit_optional(v, &has_fd, "fd", NULL);
    if (has_fd) {
        visit_type_int(v, &fd, "fd", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_fdsets(FdsetInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_FdsetInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_FdsetInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_fdsets(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    FdsetInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_fdsets(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_fdsets(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_target(TargetInfo *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_TargetInfo(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TargetInfo(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_target(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    TargetInfo *retval = NULL;

    (void)args;
    retval = qmp_query_target(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_target(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_send_key(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    KeyValueList *keys = NULL;
    bool has_hold_time = false;
    int64_t hold_time = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_KeyValueList(v, &keys, "keys", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_hold_time, "hold-time", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_hold_time) {
        visit_type_int(v, &hold_time, "hold-time", &local_err);
        if (local_err) {
            goto out;
        }
    }

    qmp_send_key(keys, has_hold_time, hold_time, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_KeyValueList(v, &keys, "keys", NULL);
    visit_optional(v, &has_hold_time, "hold-time", NULL);
    if (has_hold_time) {
        visit_type_int(v, &hold_time, "hold-time", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_screendump(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *filename = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &filename, "filename", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_screendump(filename, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &filename, "filename", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_chardev_add(ChardevReturn *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_ChardevReturn(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ChardevReturn(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_chardev_add(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    ChardevReturn *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *id = NULL;
    ChardevBackend *backend = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }
    visit_type_ChardevBackend(v, &backend, "backend", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_chardev_add(id, backend, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_chardev_add(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &id, "id", NULL);
    visit_type_ChardevBackend(v, &backend, "backend", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_chardev_remove(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *id = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &id, "id", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_chardev_remove(id, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &id, "id", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_tpm_models(TpmModelList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_TpmModelList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TpmModelList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_tpm_models(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    TpmModelList *retval = NULL;

    (void)args;
    retval = qmp_query_tpm_models(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_tpm_models(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_tpm_types(TpmTypeList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_TpmTypeList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TpmTypeList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_tpm_types(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    TpmTypeList *retval = NULL;

    (void)args;
    retval = qmp_query_tpm_types(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_tpm_types(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_tpm(TPMInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_TPMInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_TPMInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_tpm(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    TPMInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_tpm(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_tpm(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_command_line_options(CommandLineOptionInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_CommandLineOptionInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_CommandLineOptionInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_command_line_options(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    CommandLineOptionInfoList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_option = false;
    char *option = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_option, "option", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_option) {
        visit_type_str(v, &option, "option", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_query_command_line_options(has_option, option, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_command_line_options(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_option, "option", NULL);
    if (has_option) {
        visit_type_str(v, &option, "option", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_rx_filter(RxFilterInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_RxFilterInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RxFilterInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_rx_filter(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    RxFilterInfoList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_name = false;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_name) {
        visit_type_str(v, &name, "name", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_query_rx_filter(has_name, name, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_rx_filter(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_name, "name", NULL);
    if (has_name) {
        visit_type_str(v, &name, "name", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_x_input_send_event(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    bool has_console = false;
    int64_t console = {0};
    InputEventList *events = NULL;

    v = qmp_input_get_visitor(mi);
    visit_optional(v, &has_console, "console", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_console) {
        visit_type_int(v, &console, "console", &local_err);
        if (local_err) {
            goto out;
        }
    }
    visit_type_InputEventList(v, &events, "events", &local_err);
    if (local_err) {
        goto out;
    }

    qmp_x_input_send_event(has_console, console, events, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_optional(v, &has_console, "console", NULL);
    if (has_console) {
        visit_type_int(v, &console, "console", NULL);
    }
    visit_type_InputEventList(v, &events, "events", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_memdev(MemdevList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MemdevList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MemdevList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_memdev(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MemdevList *retval = NULL;

    (void)args;
    retval = qmp_query_memdev(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_memdev(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_memory_devices(MemoryDeviceInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_MemoryDeviceInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_MemoryDeviceInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_memory_devices(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    MemoryDeviceInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_memory_devices(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_memory_devices(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_acpi_ospm_status(ACPIOSTInfoList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_ACPIOSTInfoList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_ACPIOSTInfoList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_acpi_ospm_status(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    ACPIOSTInfoList *retval = NULL;

    (void)args;
    retval = qmp_query_acpi_ospm_status(&local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_acpi_ospm_status(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);

}

void qmp_marshal_input_rtc_reset_reinjection(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;

    (void)args;
    qmp_rtc_reset_reinjection(&local_err);
    error_propagate(errp, local_err);

}

static void qmp_marshal_output_query_rocker(RockerSwitch *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_RockerSwitch(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerSwitch(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_rocker(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    RockerSwitch *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_query_rocker(name, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_rocker(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_rocker_ports(RockerPortList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_RockerPortList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerPortList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_rocker_ports(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    RockerPortList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }

    retval = qmp_query_rocker_ports(name, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_rocker_ports(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_rocker_of_dpa_flows(RockerOfDpaFlowList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_RockerOfDpaFlowList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaFlowList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_rocker_of_dpa_flows(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    RockerOfDpaFlowList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;
    bool has_tbl_id = false;
    uint32_t tbl_id = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_tbl_id, "tbl-id", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_tbl_id) {
        visit_type_uint32(v, &tbl_id, "tbl-id", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_query_rocker_of_dpa_flows(name, has_tbl_id, tbl_id, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_rocker_of_dpa_flows(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    visit_optional(v, &has_tbl_id, "tbl-id", NULL);
    if (has_tbl_id) {
        visit_type_uint32(v, &tbl_id, "tbl-id", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

static void qmp_marshal_output_query_rocker_of_dpa_groups(RockerOfDpaGroupList *ret_in, QObject **ret_out, Error **errp)
{
    Error *local_err = NULL;
    QmpOutputVisitor *mo = qmp_output_visitor_new();
    QapiDeallocVisitor *md;
    Visitor *v;

    v = qmp_output_get_visitor(mo);
    visit_type_RockerOfDpaGroupList(v, &ret_in, "unused", &local_err);
    if (local_err) {
        goto out;
    }
    *ret_out = qmp_output_get_qobject(mo);

out:
    error_propagate(errp, local_err);
    qmp_output_visitor_cleanup(mo);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_RockerOfDpaGroupList(v, &ret_in, "unused", NULL);
    qapi_dealloc_visitor_cleanup(md);
}

void qmp_marshal_input_query_rocker_of_dpa_groups(QDict *args, QObject **ret, Error **errp)
{
    Error *local_err = NULL;
    RockerOfDpaGroupList *retval = NULL;
    QmpInputVisitor *mi = qmp_input_visitor_new_strict(QOBJECT(args));
    QapiDeallocVisitor *md;
    Visitor *v;
    char *name = NULL;
    bool has_type = false;
    uint8_t type = {0};

    v = qmp_input_get_visitor(mi);
    visit_type_str(v, &name, "name", &local_err);
    if (local_err) {
        goto out;
    }
    visit_optional(v, &has_type, "type", &local_err);
    if (local_err) {
        goto out;
    }
    if (has_type) {
        visit_type_uint8(v, &type, "type", &local_err);
        if (local_err) {
            goto out;
        }
    }

    retval = qmp_query_rocker_of_dpa_groups(name, has_type, type, &local_err);
    if (local_err) {
        goto out;
    }

    qmp_marshal_output_query_rocker_of_dpa_groups(retval, ret, &local_err);

out:
    error_propagate(errp, local_err);
    qmp_input_visitor_cleanup(mi);
    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_str(v, &name, "name", NULL);
    visit_optional(v, &has_type, "type", NULL);
    if (has_type) {
        visit_type_uint8(v, &type, "type", NULL);
    }
    qapi_dealloc_visitor_cleanup(md);
}

